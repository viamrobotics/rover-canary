package main

import (
	"context"
	"fmt"
	"math"
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

	// MoveStraight: distance = 100 mm, speed = 50 mm/sec
	if err := moveStraightTest(base, odometry, 100, 50, false); err != nil {
		return fmt.Errorf("error moving straight for 100 mm at 50 mm/sec, err = %v", err)
	}

	// MoveStraight: distance = -100 mm, speed = 300 mm/sec
	if err := moveStraightTest(base, odometry, -100, 300, false); err != nil {
		return fmt.Errorf("error moving straight for 1100 mm at 300 mm/sec, err = %v", err)
	}

	// MoveStraight: distance = 1000 mm, speed = -50 mm/sec
	if err := moveStraightTest(base, odometry, 1000, -50, true); err != nil {
		return fmt.Errorf("error moving straight for 1000 mm at -50 mm/sec, err = %v", err)
	}

	// MoveStraight: distance = -1000 mm, speed = -300 mm/sec
	if err := moveStraightTest(base, odometry, -1000, -300, true); err != nil {
		return fmt.Errorf("error moving straight for -1000 mm at -300 mm/sec, err = %v", err)
	}

	// Spin: distance = 15 deg, speed = 15 deg/sec
	if err := spinTest(base, odometry, 15, 15, true); err != nil {
		return fmt.Errorf("error spinning for 15 deg at 15 deg/sec, err = %v", err)
	}

	// Spin: distance = 15 deg, speed = 90 deg/sec
	if err := spinTest(base, odometry, 15, -90, true); err != nil {
		return fmt.Errorf("error spinning for 15 deg at -90 deg/sec, err = %v", err)
	}

	// Spin: distance = -360 deg, speed = 15 deg/sec
	if err := spinTest(base, odometry, -360, 15, true); err != nil {
		return fmt.Errorf("error spinning for -360 deg at 15 deg/sec, err = %v", err)
	}

	// Spin: distance = -360 deg, speed = -90 deg/sec
	if err := spinTest(base, odometry, -360, -90, true); err != nil {
		return fmt.Errorf("error spinning for -360 deg at -90 deg/sec, err = %v", err)
	}

	// SetPower: power = 10% / Stop
	if err := setPowerTest(base, 0.1); err != nil {
		return fmt.Errorf("error setting power, err = %v", err)
	}

	// SetPower: power = 90% / Stop
	if err := setPowerTest(base, 0.9); err != nil {
		return fmt.Errorf("error setting power, err = %v", err)
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

	linearErr := 10.0
	if linear.Y != 0.0 {
		linearErr = math.Abs(linear.Y) * 0.3
	}

	angularErr := 10.0
	if angular.Z != 0.0 {
		angularErr = math.Abs(angular.Z) * 0.3
	}

	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear.Y, avgLinear*1000, linearErr) || !rdkutils.Float64AlmostEqual(angular.Z, avgAngular, angularErr) {
		return fmt.Errorf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", avgLinear*1000, avgAngular, linear.Y, angular.Z)
	}
	return nil
}

func moveStraightTest(base base.Base, odometry movementsensor.MovementSensor, distance, speed float64, testSpeed bool) error {
	dir := sign(distance * speed)
	startPos, _, err := odometry.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	var linVel float64
	if testSpeed {
		go func() error {
			time.Sleep(1 * time.Second)
			timeEst := math.Abs(distance / speed)
			// sample linear velocity
			for i := 0.0; i < timeEst; i++ {
				linearVelocity, err := odometry.LinearVelocity(context.Background(), nil)
				if err != nil {
					return err
				}
				// check if desired speed has been reached
				if rdkutils.Float64AlmostEqual(linearVelocity.Y*1000.0, math.Abs(speed)*dir, 10) {
					linVel = linearVelocity.Y
					return nil
				}
				time.Sleep(1 * time.Second)
			}
			return nil
		}()
	}

	if err := base.MoveStraight(context.Background(), int(distance), speed, nil); err != nil {
		return err
	}

	endPos, _, err := odometry.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := startPos.GreatCircleDistance(endPos) * 1000000.0

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist*dir, distance, math.Abs(distance*0.3)) {
		return fmt.Errorf("measured distance %v did not equal requested distance %v", totalDist*dir, distance)
	}

	if testSpeed {
		// verify speed is approximately requested speed
		if !rdkutils.Float64AlmostEqual(linVel*1000, math.Abs(speed)*dir, math.Abs(speed*0.3)) {
			return fmt.Errorf("measured speed %v did not equal requested speed %v", linVel*1000, speed)
		}
	}

	return nil
}

func spinTest(base base.Base, odometry movementsensor.MovementSensor, distance, speed float64, testSpeed bool) error {
	dir := sign(distance * speed)
	startPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return err
	}

	var angVel float64
	if testSpeed {
		go func() error {
			time.Sleep(1 * time.Second)
			timeEst := math.Abs(distance / speed)
			// sample angular velocity
			for i := 0.0; i < timeEst; i++ {
				angularVelocity, err := odometry.AngularVelocity(context.Background(), nil)
				if err != nil {
					return err
				}
				// check if desired speed has been reached
				if rdkutils.Float64AlmostEqual(angularVelocity.Z*1000.0, math.Abs(speed)*dir, 10) {
					angVel = angularVelocity.Z
					return nil
				}
				time.Sleep(1 * time.Second)
			}
			return nil
		}()
	}

	if err := base.Spin(context.Background(), distance, speed, nil); err != nil {
		return err
	}

	endPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := endPos.OrientationVectorDegrees().Theta - startPos.OrientationVectorDegrees().Theta

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, distance, math.Abs(distance*0.3)) {
		return fmt.Errorf("measured distance %v did not equal requested distance %v", totalDist, distance)
	}

	if testSpeed {
		// verify speed is approximately requested speed
		if !rdkutils.Float64AlmostEqual(angVel, math.Abs(speed)*dir, math.Abs(speed*0.3)) {
			return fmt.Errorf("measured speed %v did not equal requested speed %v", angVel, speed)
		}
	}

	return nil
}

func setPowerTest(base base.Base, power float64) error {
	if err := base.SetPower(context.Background(), r3.Vector{Y: power}, r3.Vector{Z: 1 - power}, nil); err != nil {
		return err
	}

	time.Sleep(1 * time.Second)
	powered, err := base.IsMoving(context.Background())
	if err != nil {
		return err
	}

	if !powered {
		return fmt.Errorf("base is not powered (linear = %v, angular = %v)", power, 1-power)
	}

	if err := base.Stop(context.Background(), nil); err != nil {
		return err
	}
	powered, err = base.IsMoving(context.Background())
	if err != nil {
		return err
	}

	if powered {
		return fmt.Errorf("base did not stop with linear = %v, angular = %v", power, 1-power)
	}

	return nil
}

func sign(num float64) float64 {
	if num < 0.0 {
		return -1.0
	}
	return 1.0
}
