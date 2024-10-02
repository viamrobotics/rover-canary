package main

import (
	"context"
	"errors"
	"fmt"
	"math"
	"net/http"
	"os"
	"os/exec"
	fileupload "rovercanary/fileUpload"
	"time"

	"go.uber.org/multierr"
	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils"
	"go.viam.com/utils/rpc"

	"go.viam.com/rdk/components/encoder"
	"go.viam.com/rdk/components/motor"
	"go.viam.com/rdk/components/movementsensor"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/components/powersensor"
	"go.viam.com/rdk/logging"
	rdkutils "go.viam.com/rdk/utils"

	"bytes"
)

var (
	logger      = logging.NewLogger("client")
	failedTests = []string{}
	posExtra    = map[string]interface{}{"return_relative_pos_m": true}
	startTime   = time.Now()
)

const (
	ticksPerRotation   = 1992.0
	wheelCircumference = 381.0
	totalTests         = 55
	tickerDuration     = 100 * time.Millisecond
	delayBetweenTests  = 1
	headerString       = "type,linveldes,angveldes,time,posX,posY,theta\n"
	// replace these constants with your machine's info before running main
	address  = "<MACHINE-ADDRESS>"
	apikeyid = "<API-KEY-ID>"
	apikey   = "<API-KEY>"
	partID   = "<PART-ID>"
	webhook  = "<WEBHOOK>"
)

type baseStruct struct {
	minLinVel float64
	minAngVel float64
	baseTests func(base.Base, movementsensor.MovementSensor, float64, float64, *os.File, *os.File)
}

func main() {
	machine, err := client.New(
		context.Background(),
		address,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			apikeyid,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apikey,
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	defer machine.Close(context.Background())

	runTests(machine)

	// remove old images before uploading new ones
	removeAllImages()

	cmd := exec.Command("python3", "plot.py")
	err = cmd.Run()
	if err != nil {
		logger.Error(err)
		return
	}

	// upload all new images
	uploadAllImages()
}

// remove each saved image from previous run
func removeAllImages() {
	errs := multierr.Combine(
		os.Remove("./savedImages/sensor_ms_pos.jpg"),
		os.Remove("./savedImages/sensor_ms_vels.jpg"),
		os.Remove("./savedImages/sensor_spin_degs.jpg"),
		os.Remove("./savedImages/sensor_sv_vels.jpg"),
		os.Remove("./savedImages/wheeled_ms_pos.jpg"),
		os.Remove("./savedImages/wheeled_ms_vels.jpg"),
		os.Remove("./savedImages/wheeled_spin_degs.jpg"),
		os.Remove("./savedImages/wheeled_sv_vels.jpg"),
		os.Remove("./savedImages/encoded_go_for_rpm.jpg"),
		os.Remove("./savedImages/encoded_go_for_pos.jpg"),
		os.Remove("./savedImages/encoded_go_to_rpm.jpg"),
		os.Remove("./savedImages/encoded_go_to_pos.jpg"),
		os.Remove("./savedImages/encoded_set_rpm_rpm.jpg"),
		os.Remove("./savedImages/controlled_go_for_rpm.jpg"),
		os.Remove("./savedImages/controlled_go_for_pos.jpg"),
		os.Remove("./savedImages/controlled_go_to_rpm.jpg"),
		os.Remove("./savedImages/controlled_go_to_pos.jpg"),
		os.Remove("./savedImages/controlled_set_rpm_rpm.jpg"),
		os.Remove("./savedImages/grid_test.jpg"),
	)
	if errs != nil {
		logger.Error(errs)
	}
}

// upload all images from current run
func uploadAllImages() {
	uploadFiles("./savedImages/sensor_ms_pos.jpg", "SENSOR-BASE", "BASE-MOVESTRAIGHT-POS")
	uploadFiles("./savedImages/sensor_ms_vels.jpg", "SENSOR-BASE", "BASE-MOVESTRAIGHT-VEL")
	uploadFiles("./savedImages/sensor_spin_degs.jpg", "SENSOR-BASE", "BASE-SPIN-DEG")
	uploadFiles("./savedImages/sensor_sv_vels.jpg", "SENSOR-BASE", "BASE-SETVEL-VELS")
	uploadFiles("./savedImages/wheeled_ms_pos.jpg", "WHEELED-BASE", "BASE-MOVESTRAIGHT-POS")
	uploadFiles("./savedImages/wheeled_ms_vels.jpg", "WHEELED-BASE", "BASE-MOVESTRAIGHT-VEL")
	uploadFiles("./savedImages/wheeled_spin_degs.jpg", "WHEELED-BASE", "BASE-SPIN-DEG")
	uploadFiles("./savedImages/wheeled_sv_vels.jpg", "WHEELED-BASE", "BASE-SETVEL-VELS")
	uploadFiles("./savedImages/encoded_go_for_rpm.jpg", "ENCODED-MOTOR", "GO-FOR-RPM")
	uploadFiles("./savedImages/encoded_go_for_pos.jpg", "ENCODED-MOTOR", "GO-FOR-POS")
	uploadFiles("./savedImages/encoded_go_to_rpm.jpg", "ENCODED-MOTOR", "GO-TO-RPM")
	uploadFiles("./savedImages/encoded_go_to_pos.jpg", "ENCODED-MOTOR", "GO-TO-POS")
	uploadFiles("./savedImages/encoded_set_rpm_rpm.jpg", "ENCODED-MOTOR", "SET-RPM")
	uploadFiles("./savedImages/controlled_go_for_rpm.jpg", "CONTROLLED-MOTOR", "GO-FOR-RPM")
	uploadFiles("./savedImages/controlled_go_for_pos.jpg", "CONTROLLED-MOTOR", "GO-FOR-POS")
	uploadFiles("./savedImages/controlled_go_to_rpm.jpg", "CONTROLLED-MOTOR", "GO-TO-RPM")
	uploadFiles("./savedImages/controlled_go_to_pos.jpg", "CONTROLLED-MOTOR", "GO-TO-POS")
	uploadFiles("./savedImages/controlled_set_rpm_rpm.jpg", "CONTROLLED-MOTOR", "SET-RPM")
	uploadFiles("./savedImages/grid_test.jpg", "SENSOR-BASE", "GRID")
}

// upload a file to viam app
func uploadFiles(filename, component, testType string) {
	img, err := os.ReadFile(filename)
	if err != nil {
		logger.Error(err)
		return
	}
	bytes := bytes.NewBuffer(img)
	fileupload.UploadJpeg(context.Background(), bytes, partID, apikey, apikeyid, component, testType, logger)
}

// create the necessary file for the current test
func initializeFiles(path string) *os.File {
	fileLocation := path
	files, _ := os.ReadDir(fileLocation)
	runNum := len(files)
	logger.Infof("number of files %d", runNum)
	filePath := fmt.Sprintf("%s/run%d.txt", fileLocation, runNum+1)
	f, err := os.OpenFile(filePath, os.O_WRONLY|os.O_CREATE, 0644)
	if err != nil {
		logger.Error(err)
		return nil
	}
	return f
}

func runTests(machine *client.RobotClient) {
	// initialize all necessary components
	var errs error
	wheeledBase, err := base.FromRobot(machine, "viam_base")
	errs = multierr.Combine(errs, err)
	sensorBase, err := base.FromRobot(machine, "sensor_base")
	errs = multierr.Combine(errs, err)
	leftMotor, err := motor.FromRobot(machine, "left")
	errs = multierr.Combine(errs, err)
	rightMotor, err := motor.FromRobot(machine, "right")
	errs = multierr.Combine(errs, err)
	leftEncoder, err := encoder.FromRobot(machine, "left-enc")
	errs = multierr.Combine(errs, err)
	rightEncoder, err := encoder.FromRobot(machine, "right-enc")
	errs = multierr.Combine(errs, err)
	odometry, err := movementsensor.FromRobot(machine, "odometry")
	errs = multierr.Combine(errs, err)
	powerSensor, err := powersensor.FromRobot(machine, "ina219")
	errs = multierr.Combine(errs, err)
	movementSensor, err := movementsensor.FromRobot(machine, "imu")
	errs = multierr.Combine(errs, err)

	if errs != nil {
		logger.Errorf("error initializing components, err = %v", errs)
		return
	}

	var sb = baseStruct{
		minLinVel: 100,
		minAngVel: 30,
		baseTests: runBaseTests,
	}

	var wb = baseStruct{
		minLinVel: 100,
		minAngVel: 30,
		baseTests: runBaseTests,
	}

	startTime = time.Now()

	f := initializeFiles("./wheeledDes")
	defer f.Close()
	f2 := initializeFiles("./wheeledData")
	defer f2.Close()

	f.WriteString(headerString)
	f2.WriteString(headerString)

	// wheeled base tests
	logger.Info("Starting wheeled base tests...")
	wb.baseTests(wheeledBase, odometry, wb.minLinVel, wb.minAngVel, f, f2)

	f3 := initializeFiles("./sensorDes")
	defer f3.Close()
	f4 := initializeFiles("./sensorData")
	defer f4.Close()

	f3.WriteString(headerString)
	f4.WriteString(headerString)

	// sensor base tests
	logger.Info("Starting sensor controlled base tests...")
	sb.baseTests(sensorBase, odometry, sb.minLinVel, sb.minAngVel, f3, f4)

	f5 := initializeFiles("./encodedDes")
	defer f5.Close()
	f6 := initializeFiles("./encodedData")
	defer f6.Close()

	f5.WriteString(headerString)
	f6.WriteString(headerString)

	// encoded motor tests
	logger.Info("Starting encoded motor tests...")
	runMotorTests(leftMotor, odometry, f5, f6)

	f7 := initializeFiles("./controlledDes")
	defer f7.Close()
	f8 := initializeFiles("./controlledData")
	defer f8.Close()

	f7.WriteString(headerString)
	f8.WriteString(headerString)

	// controlled motor tests
	logger.Info("Starting controlled motor tests...")
	runMotorTests(rightMotor, odometry, f7, f8)

	// single encoder tests
	logger.Info("Starting encoder tests...")
	runEncoderTests(leftMotor, leftEncoder)
	runEncoderTests(rightMotor, rightEncoder)

	// power sensor tests
	logger.Info("Starting power sensor tests...")
	runPowerSensorTests(powerSensor)

	// movement sensor tests
	logger.Info("Starting movement sensor tests...")
	runMovementSensorTests(movementSensor)

	f9 := initializeFiles("./gridDes")
	defer f9.Close()
	f10 := initializeFiles("./gridData")
	defer f10.Close()

	f9.WriteString(headerString)
	f10.WriteString(headerString)

	// grid tests
	logger.Info("Starting grid test with sensor controlled base...")
	runGridTest(sensorBase, odometry, f9, f10)

	if len(failedTests) != 0 {
		message := "tests failed: " + fmt.Sprint(len(failedTests)) + "/" + fmt.Sprint(totalTests) + "\n"
		for i := 0; i < len(failedTests); i++ {
			message += "- " + (failedTests[i] + "\n")
		}
		sendSlackMessage(message)
	}
}

func runBaseTests(b base.Base, odometry movementsensor.MovementSensor, minLinVel, minAngVel float64, f, f2 *os.File) {
	// SetVelocity: linear = minLinVel mm/s, angular = 0 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: minLinVel}, r3.Vector{}, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetVelocity: linear = 250 mm/s, angular = 0 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: -250.0}, r3.Vector{}, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetVelocity: linear = 0 mm/s, angular = minAngVel deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{}, r3.Vector{Z: -minAngVel}, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetVelocity: linear = 0 mm/s, angular = 90 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{}, r3.Vector{Z: 90.0}, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetVelocity: linear = 200 mm/s, angular = 45 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: 200.0}, r3.Vector{Z: 45.0}, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetVelocity: linear = 200 mm/s -> linear = minLinVel mm/s
	if err := consecutiveVelocityTest(b, odometry, r3.Vector{Y: 200.0}, r3.Vector{Y: minLinVel}, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// MoveStraight: distance = 100 mm, speed = 50 mm/sec
	if err := moveStraightTest(b, odometry, 100, 50, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// MoveStraight: distance = -100 mm, speed = 250 mm/sec
	if err := moveStraightTest(b, odometry, -100, 250, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// MoveStraight: distance = 1000 mm, speed = -50 mm/sec
	if err := moveStraightTest(b, odometry, 1000, -50, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// MoveStraight: distance = -1000 mm, speed = -250 mm/sec
	if err := moveStraightTest(b, odometry, -1000, -250, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// Spin: distance = 40 deg, speed = 20 deg/sec
	if err := spinTest(b, odometry, 40, 20, false, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// Spin: distance = 40 deg, speed = -60 deg/sec
	if err := spinTest(b, odometry, 40, -60, false, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// Spin: distance = -360 deg, speed = 20 deg/sec
	if err := spinTest(b, odometry, -360, 20, true, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// Spin: distance = -360 deg, speed = -90 deg/sec
	if err := spinTest(b, odometry, -360, -90, true, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetPower: power = 10% / Stop
	if err := baseSetPowerTest(b, odometry, 0.1); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetPower: power = 90% / Stop
	if err := baseSetPowerTest(b, odometry, 0.9); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetPower: power = -50% / Stop
	if err := baseSetPowerTest(b, odometry, -0.5); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

}

func runMotorTests(m motor.Motor, odometry movementsensor.MovementSensor, f, f2 *os.File) {
	// GoFor: distance = 1 rev, speed = 10 rpm
	if err := goForTest(m, odometry, 10, 1, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// GoFor: distance = -5 rev, speed = 50 rpm
	if err := goForTest(m, odometry, 50, -5, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// GoFor: distance = 5 rev, speed = -10 rpm
	if err := goForTest(m, odometry, -10, 5, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// GoFor: distance = -5 rev, speed = -50 rpm
	if err := goForTest(m, odometry, -50, -5, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// GoTo: position = -5, speed = 50 rpm, ResetZeroPosition: offset = -2
	if err := goToTest(m, odometry, 50, -5, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// GoTo: position = 0, speed = 10 rpm
	if err := goToTest(m, odometry, 10, 0, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetRPM: speed = 10 rpm
	if err := setRPMTest(m, odometry, 10, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetRPM: speed = -50 rpm
	if err := setRPMTest(m, odometry, -50, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetRPM: rpm = 30 -> rpm = 60
	if err := consecutiveRPMTest(m, odometry, 30, 60, f, f2); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetPower: power = 10% / Stop
	if err := motorSetPowerTest(m, 0.1); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)

	// SetPower: power = -90% / Stop
	if err := motorSetPowerTest(m, -0.9); err != nil {
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), err))
	}
	time.Sleep(delayBetweenTests * time.Second)
}

func runEncoderTests(m motor.Motor, enc encoder.Encoder) {
	// reset motor position to match encoder position
	if err := m.ResetZeroPosition(context.Background(), 0, nil); err != nil {
		logger.Error(err)
		return
	}

	// motor position
	pos, err := m.Position(context.Background(), nil)
	if err != nil {
		logger.Error(err)
	}

	// encoder positon
	ticks, _, err := enc.Position(context.Background(), 0, nil)
	if err != nil {
		logger.Error(err)
	}

	// verify ticks is approximately motor position
	if !rdkutils.Float64AlmostEqual(ticks, pos*ticksPerRotation, 10) {
		storeErr := fmt.Errorf("measured encoder position %v did not equal motor position %v", ticks, pos*ticksPerRotation)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", enc.Name().ShortName(), storeErr))
	}

	// reset position
	if err := enc.ResetPosition(context.Background(), nil); err != nil {
		logger.Error(err)
	}

	// motor position
	pos, err = m.Position(context.Background(), nil)
	if err != nil {
		logger.Error(err)
	}

	// encoder positon
	ticks, _, err = enc.Position(context.Background(), 0, nil)
	if err != nil {
		logger.Error(err)
	}

	// verify ticks and motor position are zero
	if ticks != pos*ticksPerRotation || ticks != 0.0 {
		storeErr := fmt.Errorf("measured encoder position %v did not equal motor position %v", ticks, pos*ticksPerRotation)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", enc.Name().ShortName(), storeErr))
	}
}

func runPowerSensorTests(ps powersensor.PowerSensor) {
	volts, _, err := ps.Voltage(context.Background(), nil)
	if err != nil {
		logger.Error(err)
	}

	current, _, err := ps.Current(context.Background(), nil)
	if err != nil {
		logger.Error(err)
	}

	power, err := ps.Power(context.Background(), nil)
	if err != nil {
		logger.Error(err)
	}

	// verify voltage is ~15.2
	if !rdkutils.Float64AlmostEqual(volts, 15.2, 1.5) {
		storeErr := fmt.Errorf("voltage does not equal 15.2, voltage = %v", volts)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", ps.Name().ShortName(), storeErr))
	}

	// verify current is ~0.29
	if !rdkutils.Float64AlmostEqual(current, 0.29, 0.15) {
		storeErr := fmt.Errorf("current does not equal 0.29, current = %v", current)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", ps.Name().ShortName(), storeErr))
	}

	// verify power is ~4.4
	if !rdkutils.Float64AlmostEqual(power, 4.4, 1.5) {
		storeErr := fmt.Errorf("power does not equal 4.4, power = %v", power)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", ps.Name().ShortName(), storeErr))
	}
}

func runMovementSensorTests(ms movementsensor.MovementSensor) {
	linearAccel, err := ms.LinearAcceleration(context.Background(), nil)
	if err != nil {
		logger.Error(err)
	}

	// verify linear acceleration is ~9.81
	if !rdkutils.Float64AlmostEqual(linearAccel.Z, 9.81, 9.81*0.5) {
		storeErr := fmt.Errorf("linear acceleration is not ~9.81, linear acceleration = %v", linearAccel.Z)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", ms.Name().ShortName(), storeErr))
	}
}

func setVelocityTest(b base.Base, odometry movementsensor.MovementSensor, linear, angular r3.Vector, des, data *os.File) error {
	setVelocityErr := fmt.Sprintf("error setting velocity to linear = %v mm/s and anguar = %v deg/sec", linear.Y, angular.Z)
	setVelocityErr += ", err = %v"
	if err := b.SetVelocity(context.Background(), linear, angular, nil); err != nil {
		return fmt.Errorf(setVelocityErr, err)
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	// goal velocity start
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v\n", "sv", linear.Y, angular.Z, time.Since(startTime).Milliseconds()))

	sampleCtx, cancel := context.WithCancel(context.Background())
	linEst, angEst := sampleEverything(sampleCtx, odometry, nil, linear.Y, angular.Z, 5, data, "sv", cancel)
	cancel()

	// goal velocity end
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v\n", "sv", linear.Y, angular.Z, time.Since(startTime).Milliseconds()))

	if err := b.Stop(context.Background(), nil); err != nil {
		return fmt.Errorf(setVelocityErr, err)
	}

	linearErr := 50.0
	if linear.Y != 0.0 {
		linearErr = math.Abs(linear.Y) * 0.5
	}

	angularErr := 15.0
	if angular.Z != 0.0 {
		angularErr = math.Abs(angular.Z) * 0.5
	}

	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear.Y, linEst, linearErr) || !rdkutils.Float64AlmostEqual(angular.Z, angEst, angularErr) {
		return fmt.Errorf(setVelocityErr, fmt.Sprintf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", linEst, angEst, linear.Y, angular.Z))
	}
	return nil
}

func consecutiveVelocityTest(b base.Base, odometry movementsensor.MovementSensor, linear1, linear2 r3.Vector, des, data *os.File) error {
	consecutiveVelErr := "error with consecutive SetVelocity calls, err = %v"
	// SetVelocity with linear1
	if err := b.SetVelocity(context.Background(), linear1, r3.Vector{}, nil); err != nil {
		return fmt.Errorf(consecutiveVelErr, err)
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	// first goal velocity
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v\n", "sv", linear1.Y, 0.0, time.Since(startTime).Milliseconds()))
	sampleCtx, cancel := context.WithCancel(context.Background())
	linEst, angEst := sampleEverything(sampleCtx, odometry, nil, linear1.Y, 0.0, 5, data, "sv", cancel)
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v\n", "sv", linear1.Y, 0.0, time.Since(startTime).Milliseconds()))

	cancel()
	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear1.Y, linEst, linear1.Y*0.5) || !rdkutils.Float64AlmostEqual(0.0, angEst, 15.0) {
		return fmt.Errorf(consecutiveVelErr, fmt.Sprintf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", linEst, angEst, linear1.Y, 0.0))
	}

	// SetVelocity with linear 2
	if err := b.SetVelocity(context.Background(), linear2, r3.Vector{}, nil); err != nil {
		return fmt.Errorf(consecutiveVelErr, err)
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	// second goal velocity
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v\n", "sv", linear2.Y, 0.0, time.Since(startTime).Milliseconds()))
	sampleCtx, cancel = context.WithCancel(context.Background())
	linEst, angEst = sampleEverything(sampleCtx, odometry, nil, linear2.Y, 0.0, 10, data, "sv", cancel)
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v\n", "sv", linear2.Y, 0.0, time.Since(startTime).Milliseconds()))

	cancel()
	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear2.Y, linEst, linear2.Y*0.5) || !rdkutils.Float64AlmostEqual(0.0, angEst, 15.0) {
		return fmt.Errorf(consecutiveVelErr, fmt.Sprintf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", linEst, angEst, linear2.Y, 0.0))
	}

	return b.Stop(context.Background(), nil)
}

func moveStraightTest(b base.Base, odometry movementsensor.MovementSensor, distance, speed float64, des, data *os.File) error {
	moveStraightErr := fmt.Sprintf("error moving straight for %v mm at %v mm/sec", distance, speed)
	moveStraightErr += ", err = %v"
	odometry.DoCommand(context.Background(), map[string]interface{}{"reset": true})
	dir := sign(distance * speed)

	startPos, _, err := odometry.Position(context.Background(), posExtra)
	if err != nil {
		return fmt.Errorf(moveStraightErr, err)
	}

	data.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "ms", 0, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "ms", 0, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	sampleCtx, cancel := context.WithCancel(context.Background())
	done := make(chan bool)
	var speedEst float64
	go func() {
		linEst, _ := sampleEverything(sampleCtx, odometry, nil, math.Abs(speed)*dir, 0.0, math.Abs(distance/speed), data, "ms", cancel)
		speedEst = linEst
		done <- true
	}()

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "ms", math.Abs(speed)*dir, 0.0, time.Since(startTime).Milliseconds(), 0, 0, 0))
	err = b.MoveStraight(context.Background(), int(distance), speed, nil)

	// call cancel so sampleEverything returns
	cancel()
	// wait for sampleEverything to actually return so speedEst is respected
	<-done

	if err != nil {
		return fmt.Errorf(moveStraightErr, err)
	}

	endPos, _, err := odometry.Position(context.Background(), posExtra)
	if err != nil {
		return fmt.Errorf(moveStraightErr, err)
	}

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "ms", math.Abs(speed)*dir, 0.0, time.Since(startTime).Milliseconds(), startPos.Lat()+math.Abs(distance)*dir, startPos.Lng(), 0))

	totalDist := startPos.GreatCircleDistance(endPos) * 10.0

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist*dir, math.Abs(distance)*dir, math.Abs(distance*0.3)) {
		return fmt.Errorf(moveStraightErr, fmt.Sprintf("measured distance %v did not equal requested distance %v", totalDist*dir, math.Abs(distance)*dir))
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(speedEst, math.Abs(speed)*dir, math.Abs(speed*0.5)) {
		return fmt.Errorf(moveStraightErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", speedEst, math.Abs(speed)*dir))
	}

	return nil
}

func spinTest(b base.Base, odometry movementsensor.MovementSensor, distance, speed float64, testSpeed bool, des, data *os.File) error {
	spinErr := fmt.Sprintf("error spinning for %v deg at %v deg/sec", distance, speed)
	spinErr += ", err = %v"
	odometry.DoCommand(context.Background(), map[string]interface{}{"reset": true})
	time.Sleep(100 * time.Millisecond)
	dir := sign(distance * speed)

	data.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "s", 0.0, 0.0, time.Since(startTime).Milliseconds(), 0.0, 0.0, 0))
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "s", 0.0, math.Abs(speed)*dir, time.Since(startTime).Milliseconds(), 0, 0, 0))

	var speedEst float64
	sampleCtx, cancel := context.WithCancel(context.Background())
	done := make(chan bool)
	go func() {
		_, angEst := sampleEverything(sampleCtx, odometry, nil, 0.0, math.Abs(speed)*dir, math.Abs(distance/speed), data, "s", cancel)
		speedEst = angEst
		done <- true
	}()

	start := time.Now()
	err := b.Spin(context.Background(), distance, speed, nil)

	// call cancel so sampleEverything returns
	cancel()
	// wait for sampleEverything to actually return so speedEst is respected
	<-done

	if err != nil {
		return fmt.Errorf(spinErr, err)
	}

	endTime := time.Now()
	endPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(spinErr, err)
	}
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "s", 0.0, math.Abs(speed)*dir, time.Since(startTime).Milliseconds(), 0.0, 0.0, rdkutils.DegToRad(distance*dir)))

	totalDist := distBetweenAngles(endPos.OrientationVectorRadians().Theta, 0, math.Abs(distance)*dir)

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, math.Abs(distance)*dir, math.Abs(distance*0.3)) {
		return fmt.Errorf(spinErr, fmt.Sprintf("measured distance %v did not equal requested distance %v", totalDist, math.Abs(distance)*dir))
	}

	if testSpeed {
		// verify speed is approximately requested speed
		if !rdkutils.Float64AlmostEqual(speedEst, math.Abs(speed)*dir, math.Abs(speed)*0.5) {
			return fmt.Errorf(spinErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", speedEst, math.Abs(speed)*dir))
		}
	} else {
		if float64(endTime.Sub(start))*1e-9 > 5 {
			return fmt.Errorf(spinErr, fmt.Sprintf("spin call took longer than expected, actual time = %v, max allowed time = %v", float64(endTime.Sub(start))*1e-9, math.Abs(distance/speed*5)))
		}
	}

	return nil
}

func baseSetPowerTest(b base.Base, odometry movementsensor.MovementSensor, power float64) error {
	powerErr := "error setting power, err = %v"
	// if power is negative, just test linear power
	angPwr := 0.0
	if power > 0 {
		angPwr = 1 - power
	}
	if err := b.SetPower(context.Background(), r3.Vector{Y: power}, r3.Vector{Z: angPwr}, nil); err != nil {
		return fmt.Errorf(powerErr, err)
	}

	// wait for base to start moving
	time.Sleep(1 * time.Second)
	powered, err := b.IsMoving(context.Background())
	if err != nil {
		return fmt.Errorf(powerErr, err)
	}

	if !powered {
		return fmt.Errorf(powerErr, fmt.Sprintf("base is not powered (linear = %v, angular = %v)", power, angPwr))
	}

	linVel, err := odometry.LinearVelocity(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(powerErr, err)
	}
	angVel, err := odometry.AngularVelocity(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(powerErr, err)
	}

	if power < 0 {
		if linVel.Y > 0 {
			return fmt.Errorf(powerErr, fmt.Sprintf("base should be moving in the negative direction (linear = %v, angular = %v)", linVel.Y, angVel.Z))
		}
	} else {
		if linVel.Y < 0 || angVel.Z < 0 {
			return fmt.Errorf(powerErr, fmt.Sprintf("base should be moving in the positive direction (linear = %v, angular = %v)", linVel.Y, angVel.Z))
		}
	}

	if err := b.Stop(context.Background(), nil); err != nil {
		return fmt.Errorf(powerErr, err)
	}
	powered, err = b.IsMoving(context.Background())
	if err != nil {
		return fmt.Errorf(powerErr, err)
	}

	if powered {
		return fmt.Errorf(powerErr, fmt.Sprintf("base did not stop with linear = %v, angular = %v", power, 1-power))
	}

	return nil
}

func goForTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm, revolutions float64, des, data *os.File) error {
	goForErr := fmt.Sprintf("error going for %v rev at %v rpm", revolutions, rpm)
	goForErr += ", err = %v"
	dir := sign(rpm * revolutions)

	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(goForErr, err)
	}

	sampleCtx, cancel := context.WithCancel(context.Background())
	done := make(chan bool)
	var rpmEst float64
	go func() {
		linEst, _ := sampleEverything(sampleCtx, odometry, &m, math.Abs(rpm)*dir, 0.0, math.Abs(revolutions/rpm*60), data, "gf", cancel)
		rpmEst = linEst
		done <- true
	}()

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gf", 0, 0, time.Since(startTime).Milliseconds(), startPos, 0, 0))
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gf", math.Abs(rpm)*dir, 0, time.Since(startTime).Milliseconds(), startPos, 0, 0))
	err = m.GoFor(context.Background(), rpm, revolutions, nil)

	// call cancel so sampleEverything returns
	cancel()
	// wait for sampleEverything to actually return so speedEst is respected
	<-done

	if err != nil {
		return fmt.Errorf(goForErr, err)
	}

	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(goForErr, err)
	}

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gf", math.Abs(rpm)*dir, 0, time.Since(startTime).Milliseconds(), startPos+(math.Abs(revolutions)*dir), 0, 0))
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gf", 0, 0, time.Since(startTime).Milliseconds(), startPos+(math.Abs(revolutions)*dir), 0, 0))

	totalDist := endPos - startPos
	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, math.Abs(revolutions)*dir, math.Abs(revolutions*0.3)) {
		return fmt.Errorf(goForErr, fmt.Sprintf("measured revolutions %v did not equal requested revolutions %v", totalDist, revolutions))
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, math.Abs(rpm)*dir, math.Abs(rpm*0.5)) {
		return fmt.Errorf(goForErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", rpmEst, math.Abs(rpm)*dir))
	}
	return nil
}

func goToTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm, position float64, des, data *os.File) error {
	goToErr := fmt.Sprintf("error going to position %v at %v rpm", position, rpm)
	goToErr += ", err = %v"
	var rpmEst float64

	if err := m.ResetZeroPosition(context.Background(), -2, nil); err != nil {
		return fmt.Errorf(goToErr, err)
	}

	// sleep for one interval of wheeled odometry polling so there isn't a large change in position over a short period of time
	time.Sleep(500 * time.Millisecond)

	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(goToErr, err)
	}

	dir := sign((position - startPos) * rpm)

	sampleCtx, cancel := context.WithCancel(context.Background())
	done := make(chan bool)
	go func() {
		linEst, _ := sampleEverything(sampleCtx, odometry, &m, math.Abs(rpm)*dir, 0.0, math.Abs((position-startPos)/rpm*60), data, "gt", cancel)
		rpmEst = linEst
		done <- true
	}()

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gt", 0, 0, time.Since(startTime).Milliseconds(), startPos, 0, 0))
	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gt", math.Abs(rpm)*dir, 0, time.Since(startTime).Milliseconds(), startPos, 0, 0))

	err = m.GoTo(context.Background(), rpm, position, nil)

	// call cancel so sampleEverything returns
	cancel()
	// wait for sampleEverything to actually return so speedEst is respected
	<-done

	if err != nil {
		return fmt.Errorf(goToErr, err)
	}

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "gt", math.Abs(rpm)*dir, 0, time.Since(startTime).Milliseconds(), position, 0, 0))

	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(goToErr, err)
	}

	// verify start position is 2 after ResetZeroPosition
	if startPos != 2 {
		return fmt.Errorf(goToErr, fmt.Sprintf("startPos = %v when it should be 2", startPos))
	}

	// verify end position is approximately requested end position
	if !rdkutils.Float64AlmostEqual(endPos, position, 1) {
		return fmt.Errorf(goToErr, fmt.Sprintf("measured end position %v did not equal requested end position %v", endPos, position))
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(math.Abs(rpmEst), math.Abs(rpm), math.Abs(rpm*0.5)) {
		return fmt.Errorf(goToErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", math.Abs(rpmEst), math.Abs(rpm)))
	}
	return nil
}

func setRPMTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm float64, des, data *os.File) error {
	setRPMErr := fmt.Sprintf("error setting rpm at %v rpm", rpm)
	setRPMErr += ", err = %v"
	if err := m.SetRPM(context.Background(), rpm, nil); err != nil {
		return fmt.Errorf(setRPMErr, err)
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "rpm", rpm, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	sampleCtx, cancel := context.WithCancel(context.Background())
	rpmEst, _ := sampleEverything(sampleCtx, odometry, &m, rpm, 0.0, 5, data, "rpm", cancel)
	cancel()

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "rpm", rpm, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	if err := m.Stop(context.Background(), nil); err != nil {
		return fmt.Errorf(setRPMErr, err)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, rpm, math.Abs(rpm)*0.5) {
		return fmt.Errorf(setRPMErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", rpmEst, rpm))
	}
	return nil
}

func consecutiveRPMTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm1, rpm2 float64, des, data *os.File) error {
	consecutiveRPMErr := "error with consecutive SetRPM calls, err = %v"
	// SetRPM with rpm1
	if err := m.SetRPM(context.Background(), rpm1, nil); err != nil {
		return fmt.Errorf(consecutiveRPMErr, err)
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "rpm", rpm1, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	sampleCtx, cancel := context.WithCancel(context.Background())
	rpmEst, _ := sampleEverything(sampleCtx, odometry, &m, rpm1, 0.0, 5, data, "rpm", cancel)
	cancel()

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "rpm", rpm1, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, rpm1, math.Abs(rpm1)*0.5) {
		return fmt.Errorf(consecutiveRPMErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", rpmEst, rpm1))
	}

	// SetRPM with rpm2
	if err := m.SetRPM(context.Background(), rpm2, nil); err != nil {
		return fmt.Errorf(consecutiveRPMErr, err)
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "rpm", rpm2, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	sampleCtx, cancel = context.WithCancel(context.Background())
	rpmEst, _ = sampleEverything(sampleCtx, odometry, &m, rpm2, 0.0, 5, data, "rpm", cancel)
	cancel()

	des.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", "rpm", rpm2, 0, time.Since(startTime).Milliseconds(), 0, 0, 0))

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, rpm2, math.Abs(rpm2)*0.5) {
		return fmt.Errorf(consecutiveRPMErr, fmt.Sprintf("measured speed %v did not equal requested speed %v", rpmEst, rpm2))
	}

	return m.Stop(context.Background(), nil)
}

func motorSetPowerTest(m motor.Motor, power float64) error {
	setPowerErr := "error setting power, err = %v"
	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(setPowerErr, err)
	}

	if err := m.SetPower(context.Background(), power, nil); err != nil {
		return fmt.Errorf(setPowerErr, err)
	}

	// wait for motor to start moving
	time.Sleep(2 * time.Second)
	powered, powerPct, err := m.IsPowered(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(setPowerErr, err)
	}

	if !powered {
		return fmt.Errorf(setPowerErr, fmt.Sprintf("motor is not powered (power = %v)", powerPct))
	}

	if !rdkutils.Float64AlmostEqual(powerPct, power, math.Abs(power)*0.3) {
		return fmt.Errorf(setPowerErr, fmt.Sprintf("measured power %v does not match requested power %v", powerPct, power))
	}

	if err := m.Stop(context.Background(), nil); err != nil {
		return fmt.Errorf(setPowerErr, err)
	}
	powered, err = m.IsMoving(context.Background())
	if err != nil {
		return fmt.Errorf(setPowerErr, err)
	}

	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return fmt.Errorf(setPowerErr, err)
	}

	if sign(endPos-startPos) != sign(power) {
		return fmt.Errorf(setPowerErr, fmt.Sprintf("motor dir = %v, requested motor dir = %v", sign(endPos-startPos), sign(power)))
	}

	if powered {
		return fmt.Errorf(setPowerErr, fmt.Sprintf("motor did not stop with power = %v", power))
	}

	return nil
}

func doMoveStraight(odometry movementsensor.MovementSensor, b base.Base, desDist, desVel float64, data *os.File) error {
	sampleCtx, cancel := context.WithCancel(context.Background())
	done := make(chan bool)
	go func() {
		_, _ = sampleEverything(sampleCtx, odometry, nil, desVel, 0.0, desDist/desVel, data, "grid", cancel)
		done <- true
	}()

	err := b.MoveStraight(context.Background(), int(desDist), desVel, nil)

	// call cancel so sampleEverything returns
	cancel()
	// wait for sampleEverything to actually return so speedEst is respected
	<-done

	return err
}

func doSpin(b base.Base, lastAng, desAng, desAngVel float64) float64 {
	lastAng += desAng
	if lastAng > 360 {
		lastAng -= 360
	}

	err := b.Spin(context.Background(), desAng, desAngVel, nil)
	if err != nil {
		logger.Error(err)
		return lastAng
	}
	time.Sleep(1 * time.Second)
	return lastAng
}

func runGridTest(b base.Base, odometry movementsensor.MovementSensor, des, data *os.File) {
	gridPath := []string{"long-straight", "left", "short-straight", "left", "long-straight", "right", "short-straight", "right", "long-straight", "left", "short-straight", "left", "long-straight"}
	odometry.DoCommand(context.Background(), map[string]interface{}{"reset": true})

	desVel := 100.0
	desDist := 0.0
	desAng := 0.0
	desAngVel := 30.0
	posLat, posLng := 0.0, 0.0
	lastAng := 0.0

	var lat, lng, desLat, desLng = []float64{}, []float64{}, []float64{}, []float64{}

	startPos, _, err := odometry.Position(context.Background(), posExtra)
	if err != nil {
		logger.Error(err)
		return
	}

	lat = append(lat, startPos.Lat())
	lng = append(lng, startPos.Lng())
	desLat = append(desLat, startPos.Lat())
	desLng = append(desLng, startPos.Lng())

	for _, s := range gridPath {
		switch s {
		case "long-straight":
			desDist = 1500.0
			posLat, posLng = writeDesired(des, posLat, posLng, lastAng, desDist)
			desLat = append(desLat, posLat/1000.0)
			desLng = append(desLng, posLng/1000.0)

			if err := doMoveStraight(odometry, b, desDist, desVel, data); err != nil {
				logger.Error(err)
				return
			}

			endPos, _, err := odometry.Position(context.Background(), posExtra)
			if err != nil {
				logger.Error(err)
				return
			}

			lat = append(lat, endPos.Lat())
			lng = append(lng, endPos.Lng())

		case "short-straight":
			desDist = 500.0
			posLat, posLng = writeDesired(des, posLat, posLng, lastAng, desDist)
			desLat = append(desLat, posLat/1000.0)
			desLng = append(desLng, posLng/1000.0)

			if err := doMoveStraight(odometry, b, desDist, desVel, data); err != nil {
				logger.Error(err)
				return
			}

			endPos, _, err := odometry.Position(context.Background(), posExtra)
			if err != nil {
				logger.Error(err)
				return
			}

			lat = append(lat, endPos.Lat())
			lng = append(lng, endPos.Lng())
		case "left":
			desAng = 90
			lastAng = doSpin(b, lastAng, desAng, desAngVel)

		case "right":
			desAng = -90
			lastAng = doSpin(b, lastAng, desAng, desAngVel)
		}
	}

	numSamples := len(lat)
	rmsNumSamples := len(desLat)
	rmsErrorSum := 0.0
	var diff float64
	for i := 0; i < numSamples; i++ {
		predictedPoint := geo.NewPoint(desLat[i]/1000.0, desLng[i])
		actualPoint := geo.NewPoint(lat[i], lng[i])
		// diff = (predicted - actual)^2
		diff = math.Pow(actualPoint.GreatCircleDistance(predictedPoint), 2)
		rmsErrorSum += diff
	}

	rmsErr := math.Sqrt(rmsErrorSum / float64(rmsNumSamples))

	if rmsErr > 150 {
		storeErr := fmt.Errorf("rms error %v is higher than the minimum allowed error %v", rmsErr, 150)
		failedTests = append(failedTests, fmt.Sprintf("grid test: %v", storeErr))
	}
}

func writeDesired(file *os.File, posLat, posLng, lastAng, desDist float64) (float64, float64) {
	// write desired path
	file.WriteString(fmt.Sprintf("%.3v,%.3v\n", posLat, posLng))
	if lastAng == 0 || lastAng == 360 {
		posLat += desDist
	} else if lastAng == 180 {
		posLat -= desDist
	} else if lastAng == -90 {
		posLng += desDist
	} else if lastAng == 90 {
		posLng -= desDist
	}
	file.WriteString(fmt.Sprintf("%.3v,%.3v\n", posLat, posLng))

	return posLat, posLng
}

func sign(num float64) float64 {
	if num < 0.0 {
		return -1.0
	}
	return 1.0
}

func distBetweenAngles(endRad, startRad, originalDist float64) float64 {
	turns := int(originalDist) / 360 % 360
	total := rdkutils.RadToDeg((math.Sin(endRad) - math.Sin(startRad))) + float64(turns)*360
	return total
}

func average(arr [5]float64) float64 {
	sum := 0.
	for _, val := range arr {
		sum += val
	}
	return sum / float64(len(arr))
}

func sampleEverything(ctx context.Context, odometry movementsensor.MovementSensor, m *motor.Motor, goalLinVel, goalAngVel, timeEst float64, data *os.File, testType string, cancel func()) (float64, float64) {
	bestLin, bestAng := 0.0, 0.0
	maxLin, maxAng := 0.0, 0.0
	prevMotorPos := 0.0
	start := time.Now()
	if m != nil {
		var err error
		prevMotorPos, err = (*m).Position(ctx, nil)
		if err != nil {
			logger.Error(err)
			return -1, -1
		}
	}
	prevTime := time.Now()
	var avgRPM [5]float64
	avgRPMIndex := 0

	for {
		if !utils.SelectContextOrWait(ctx, tickerDuration) {
			break
		}
		if cancel == nil {
			break
		}

		// motor tests
		if testType == "rpm" || testType == "gt" || testType == "gf" {
			if m == nil {
				logger.Error("provide a valid motor")
				return -1, -1
			}
			motorPos, err := (*m).Position(ctx, nil)
			if err != nil {
				if errors.Is(err, context.Canceled) {
					break
				}
				logger.Error(err)
				return -1, -1
			}
			currTime := time.Now()
			avgRPM[avgRPMIndex%5] = (motorPos - prevMotorPos) / currTime.Sub(prevTime).Minutes()
			motorRPM := average(avgRPM)
			data.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", testType, avgRPM[avgRPMIndex%5], prevMotorPos, time.Since(startTime).Milliseconds(), motorPos, 0, 0))
			prevMotorPos = motorPos
			prevTime = currTime
			avgRPMIndex++

			// calculate rpm error margins
			rpmErr := 0.0
			if goalLinVel != 0 {
				rpmErr = goalLinVel * 0.5
			}

			// check if motorRPM within the error margin of the goalLinVel
			if rdkutils.Float64AlmostEqual(goalLinVel, motorRPM, rpmErr) {
				bestLin = motorRPM
			}

			// check if the current motorRPM greater than the current max rpm
			if (goalLinVel < 0 && motorRPM < maxLin) || (goalLinVel > 0 && motorRPM > maxLin) {

				maxLin = motorRPM
			}

		} else { // base tests
			pos, _, err := odometry.Position(ctx, posExtra)
			if err != nil {
				if errors.Is(err, context.Canceled) {
					break
				}
				logger.Error(err)
				return -1, -1
			}
			linVel, err := odometry.LinearVelocity(ctx, nil)
			if err != nil {
				if errors.Is(err, context.Canceled) {
					break
				}
				logger.Error(err)
				return -1, -1
			}
			angVel, err := odometry.AngularVelocity(ctx, nil)
			if err != nil {
				if errors.Is(err, context.Canceled) {
					break
				}
				logger.Error(err)
				return -1, -1
			}
			angle, err := odometry.Orientation(ctx, nil)
			if err != nil {
				if errors.Is(err, context.Canceled) {
					break
				}
				logger.Error(err)
				return -1, -1
			}
			data.WriteString(fmt.Sprintf("%v,%.3v,%.3v,%v,%.3v,%.3v,%.3v\n", testType, linVel.Y*1000, angVel.Z, time.Since(startTime).Milliseconds(), pos.Lat(), pos.Lng(), angle.OrientationVectorRadians().Theta))

			// calculate linear and angular error margins
			linErr, angErr := 50.0, 15.0
			if goalLinVel != 0 {
				linErr = math.Abs(goalLinVel) * 0.5
			}
			if goalAngVel != 0 {
				angErr = math.Abs(goalAngVel) * 0.5
			}

			// check if linVel and angVel are within the error margin of the goalLinVel and goalAngVel, respectively
			if rdkutils.Float64AlmostEqual(goalLinVel, linVel.Y*1000, linErr) {
				bestLin = linVel.Y * 1000
			}
			if rdkutils.Float64AlmostEqual(goalAngVel, angVel.Z, angErr) {
				bestAng = angVel.Z
			}

			// check if the current linVel and angVel are greater than the current max lin and ang vel
			if (goalLinVel < 0 && linVel.Y*1000 < maxLin) || (goalLinVel > 0 && linVel.Y*1000 > maxLin) {
				maxLin = linVel.Y * 1000
			}
			if (goalAngVel < 0 && angVel.Z < maxAng) || (goalAngVel > 0 && angVel.Z > maxAng) {
				maxAng = angVel.Z
			}
		}

		// check if the max time allowed has passed
		timeElapsed := float64(time.Since(start)) * 1e-9
		if float64(timeElapsed) > timeEst*2 {
			break
		}
	}
	if bestLin == 0 {
		bestLin = maxLin
	}
	if bestAng == 0 {
		bestAng = maxAng
	}
	return bestLin, bestAng
}

func sendSlackMessage(msg string) {
	data := []byte("{'text': '" + msg + "'}")
	body := bytes.NewReader(data)

	req, err := http.NewRequest("POST", webhook, body)
	if err != nil {
		logger.Error(err)
	}
	req.Header.Set("Content-Type", "application/json")

	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		logger.Error(err)
	}
	defer resp.Body.Close()
}
