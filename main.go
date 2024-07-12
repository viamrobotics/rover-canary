package main

import (
	"context"
	"fmt"
	"time"

	"go.uber.org/multierr"
	"go.viam.com/rdk/components/base"

	// "go.viam.com/rdk/components/encoder"
	// "go.viam.com/rdk/components/motor"
	"go.viam.com/rdk/components/movementsensor"
	// "go.viam.com/rdk/components/powersensor"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"
)

var logger = logging.NewDebugLogger("client")

func main() {
	machine, err := client.New(
		context.Background(),
		"rover-canary-main.hjyd54x654.viam.cloud",
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			"a14bc023-9ee8-4a29-bd61-597a0fe11f97",
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: "d8nu8bltkbvlx2fini9tjj08sjmno7sd",
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	defer machine.Close(context.Background())

	// initialize all necessary components
	var errs error
	wheeledBase, err := base.FromRobot(machine, "viam_base")
	errs = multierr.Combine(errs, err)
	sensorBase, err := base.FromRobot(machine, "sensor_base")
	errs = multierr.Combine(errs, err)
	// leftMotor, err := motor.FromRobot(machine, "left")
	// errs = multierr.Combine(errs, err)
	// rightMotor, err := motor.FromRobot(machine, "right")
	// errs = multierr.Combine(errs, err)
	// leftEncoder, err := encoder.FromRobot(machine, "left-enc")
	// errs = multierr.Combine(errs, err)
	// rightEncoder, err := encoder.FromRobot(machine, "right-enc")
	// errs = multierr.Combine(errs, err)
	odometry, err := movementsensor.FromRobot(machine, "odometry")
	errs = multierr.Combine(errs, err)
	// powerSensor, err := powersensor.FromRobot(machine, "ina219")
	// errs = multierr.Combine(errs, err)

	if errs != nil {
		logger.Errorf("error initializing components, err = %v", errs)
		return
	}

	// sensor base tests
	logger.Info("Starting sensor controlled base tests...")
	if err := runBaseTests(sensorBase, odometry); err != nil {
		logger.Error(err)
		return
	}

	// wheeled base tests
	logger.Info("Starting wheeled base tests...")
	if err := runBaseTests(wheeledBase, odometry); err != nil {
		logger.Error(err)
		return
	}
}

func runBaseTests(base base.Base, odometry movementsensor.MovementSensor) error {
	// SetVelocity: linear = 50 mm/s, angular = 0 deg/sec
	if err := setVelocityTest(base, odometry, r3.Vector{Y: 50.0}, r3.Vector{}); err != nil {
		return fmt.Errorf("error setting velocity to linear = 50 mm/s and anguar = 0 deg/sec, err = %v", err)
	}

	// SetVelocity: linear = 250 mm/s, angular = 0 deg/sec
	if err := setVelocityTest(base, odometry, r3.Vector{Y: -250.0}, r3.Vector{}); err != nil {
		return fmt.Errorf("error setting velocity to linear = -250 mm/s and anguar = 0 deg/sec, err = %v", err)
	}

	// SetVelocity: linear = 0 mm/s, angular = 10 deg/sec
	if err := setVelocityTest(base, odometry, r3.Vector{}, r3.Vector{Z: -15.0}); err != nil {
		return fmt.Errorf("error setting velocity to linear = 0 mm/s and anguar = -15 deg/sec, err = %v", err)
	}

	// SetVelocity: linear = 0 mm/s, angular = 90 deg/sec
	if err := setVelocityTest(base, odometry, r3.Vector{}, r3.Vector{Z: 90.0}); err != nil {
		return fmt.Errorf("error setting velocity to linear = 0 mm/s and anguar = 90 deg/sec, err = %v", err)
	}

	return nil
}

func setVelocityTest(base base.Base, odometry movementsensor.MovementSensor, linear, angular r3.Vector) error {
	if err := base.SetVelocity(context.Background(), linear, angular, nil); err != nil {
		return err
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)
	linearSum, angularSum := 0.0, 0.0

	// sample linear velocity for 5 seconds
	startTime := time.Now()
	for i := 0; i < 10; i++ {
		time.Sleep(1 * time.Second)

		linearVelocity, err := odometry.LinearVelocity(context.Background(), nil)
		if err != nil {
			return err
		}
		linearSum += linearVelocity.Y

		angularVelocity, err := odometry.AngularVelocity(context.Background(), nil)
		if err != nil {
			return err
		}
		angularSum += angularVelocity.Z
	}
	endTime := time.Now()

	if err := base.Stop(context.Background(), nil); err != nil {
		return err
	}

	totalTime := float64(endTime.Sub(startTime)) * 1e-9
	if totalTime == 0 {
		return fmt.Errorf("error with time, totalTime = %v", totalTime)
	}
	avgLinear := linearSum / totalTime
	avgAngular := angularSum / totalTime

	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear.Y, avgLinear*1000, 25) || !rdkutils.Float64AlmostEqual(angular.Z, avgAngular, 10) {
		return fmt.Errorf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", avgLinear*1000, avgAngular, linear.Y, linear.Z)
	}
	return nil
}
