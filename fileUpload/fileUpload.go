package fileupload

import (
	"bytes"
	"context"
	"io"
	"net/url"
	"path/filepath"

	"github.com/pkg/errors"
	pbDataSync "go.viam.com/api/app/datasync/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/utils/rpc"
)

const (
	UploadChunkSize = 32 * 1024 // size of the data included in each message of a FileUpload stream.
	imageName       = "preview.jpeg"
	appURL          = "https://app.viam.com:443"
)

// uploadJpeg uploads a jpeg thumbnail of a pointcloud used for a slam map package.
func UploadJpeg(
	ctx context.Context,
	content *bytes.Buffer,
	partID string, // you must have the partID for the robot
	apiKey, apiKeyID string, // i think the robot api key you already use should work, but if it doesn't we can easily up the permissions on that key
	logger logging.Logger,
) (string, error) {
	syncClient, conn, err := connectToApp(ctx, apiKey, apiKeyID, logger)
	defer conn.Close()

	stream, err := syncClient.FileUpload(ctx)
	if err != nil {
		return "", err
	}
	md := &pbDataSync.UploadMetadata{
		// NOTE: Passing the PartID is temp.
		// Once we move to use Org Keys for authenticating with App
		// the PartID field can be removed in favor of sending the
		// OrgKey
		PartId:        partID,
		Type:          pbDataSync.DataType_DATA_TYPE_FILE, // i used this in the past and saw my images show up in data, but it is possible this is no longer correct
		FileName:      filepath.Base(imageName),
		FileExtension: filepath.Ext(imageName),
		Tags: []string{
			"TAG1", // generic for all canary uploads
			"TAG2", // specific for this run
		},
	}

	// Send metadata FileUploadRequest.
	req := &pbDataSync.FileUploadRequest{
		UploadPacket: &pbDataSync.FileUploadRequest_Metadata{
			Metadata: md,
		},
	}
	if err := stream.Send(req); err != nil {
		return "", err
	}

	if err := sendFileUploadRequests(ctx, stream, content); err != nil {
		return "", errors.Wrap(err, "error syncing image")
	}

	res, err := stream.CloseAndRecv()
	if err != nil {
		return "", errors.Wrap(err, "received error response while syncing image")
	}

	return res.GetFileId(), nil
}

func connectToApp(ctx context.Context, apiKey, apiKeyID string, logger logging.Logger) (pbDataSync.DataSyncServiceClient, rpc.ClientConn, error) {
	u, err := url.Parse(appURL)
	if err != nil {
		return nil, nil, err
	}

	opts := rpc.WithEntityCredentials(
		apiKeyID,
		rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: apiKey,
		})

	conn, err := rpc.DialDirectGRPC(ctx, u.Host, logger.AsZap(), opts)
	if err != nil {
		return nil, nil, err
	}
	return pbDataSync.NewDataSyncServiceClient(conn), conn, nil
}

func readNextFileUploadFileChunk(f *bytes.Buffer) (*pbDataSync.FileData, error) {
	byteArr := make([]byte, UploadChunkSize)
	numBytesRead, err := f.Read(byteArr)
	if err != nil {
		return nil, err
	}
	if numBytesRead < UploadChunkSize {
		byteArr = byteArr[:numBytesRead]
	}
	return &pbDataSync.FileData{Data: byteArr}, nil
}

// getNextFileUploadRequest gets the next chunk of a file upload for data sync.
func getNextFileUploadRequest(ctx context.Context, f *bytes.Buffer) (*pbDataSync.FileUploadRequest, error) {
	select {
	case <-ctx.Done():
		return nil, context.Canceled
	default:
		// Get the next file data reading from file, check for an error.
		next, err := readNextFileUploadFileChunk(f)
		if err != nil {
			return nil, err
		}
		// Otherwise, return an UploadRequest and no error.
		return &pbDataSync.FileUploadRequest{
			UploadPacket: &pbDataSync.FileUploadRequest_FileContents{
				FileContents: next,
			},
		}, nil
	}
}

// sendFIleUploadRequests sends a file upload to app in a series of chunks.
func sendFileUploadRequests(ctx context.Context, stream pbDataSync.DataSyncService_FileUploadClient, f *bytes.Buffer) error {
	// Loop until there is no more content to be read from file.
	for {
		select {
		case <-ctx.Done():
			return context.Canceled
		default:
			// Get the next UploadRequest from the file.
			uploadReq, err := getNextFileUploadRequest(ctx, f)

			// EOF means we've completed successfully.
			if errors.Is(err, io.EOF) {
				return nil
			}

			if err != nil {
				return err
			}

			if err = stream.Send(uploadReq); err != nil {
				return err
			}
		}
	}
}
