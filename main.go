package main

import (
	"context"
	"fmt"
	"math"
	"time"

	"go.uber.org/multierr"
	"go.viam.com/rdk/components/base"

	"go.viam.com/rdk/components/encoder"
	"go.viam.com/rdk/components/motor"
	"go.viam.com/rdk/components/movementsensor"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/components/powersensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"

	"bytes"
	"net/http"
)

var (
	logger             = logging.NewLogger("client")
	ticksPerRotation   = 1992.0
	failedTests        = []string{}
	wheelCircumference = 381.0
	totalTests         = 55
	posExtra           = map[string]interface{}{"return_relative_pos_m": true}
)

type baseStruct struct {
	minLinVel float64
	minAngVel float64
	baseTests func(base.Base, movementsensor.MovementSensor, float64, float64)
}

func main() {
	machine, err := client.New(
		context.Background(),
		"<MACHINE-ADDRESS>",
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			"<API-KEY-ID>",
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: "<API-KEY>",
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	defer machine.Close(context.Background())

	runTests(machine)
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
		minLinVel: 50,
		minAngVel: 15,
		baseTests: runBaseTests,
	}

	var wb = baseStruct{
		minLinVel: 100,
		minAngVel: 30,
		baseTests: runBaseTests,
	}

	// wheeled base tests
	logger.Info("Starting wheeled base tests...")
	wb.baseTests(wheeledBase, odometry, wb.minLinVel, wb.minAngVel)

	// sensor base tests
	logger.Info("Starting sensor controlled base tests...")
	sb.baseTests(sensorBase, odometry, sb.minLinVel, sb.minAngVel)

	// encoded motor tests
	logger.Info("Starting encoded motor tests...")
	runMotorTests(leftMotor, odometry)

	// controlled motor tests
	logger.Info("Starting controlled motor tests...")
	runMotorTests(rightMotor, odometry)

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

	// grid tests
	logger.Info("Starting grid test with sensor controlled base...")
	runGridTest(sensorBase, odometry)

	if len(failedTests) != 0 {
		message := "tests failed: " + fmt.Sprint(len(failedTests)) + "/" + fmt.Sprint(totalTests) + "\n"
		for i := 0; i < len(failedTests); i++ {
			message += "- " + (failedTests[i] + "\n")
		}
		sendSlackMessage(message)
	}
}

func runBaseTests(b base.Base, odometry movementsensor.MovementSensor, minLinVel, minAngVel float64) {
	// SetVelocity: linear = minLinVel mm/s, angular = 0 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: minLinVel}, r3.Vector{}); err != nil {
		storeErr := fmt.Errorf("error setting velocity to linear = %v mm/s and anguar = 0 deg/sec, err = %v", minLinVel, err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetVelocity: linear = 250 mm/s, angular = 0 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: -250.0}, r3.Vector{}); err != nil {
		storeErr := fmt.Errorf("error setting velocity to linear = -250 mm/s and anguar = 0 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetVelocity: linear = 0 mm/s, angular = minAngVel deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{}, r3.Vector{Z: -minAngVel}); err != nil {
		storeErr := fmt.Errorf("error setting velocity to linear = 0 mm/s and anguar = %v deg/sec, err = %v", -minAngVel, err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetVelocity: linear = 0 mm/s, angular = 90 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{}, r3.Vector{Z: 90.0}); err != nil {
		storeErr := fmt.Errorf("error setting velocity to linear = 0 mm/s and anguar = 90 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetVelocity: linear = 200 mm/s, angular = 45 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: 200.0}, r3.Vector{Z: 45.0}); err != nil {
		storeErr := fmt.Errorf("error setting velocity to linear = 200 mm/s and anguar = 45 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetVelocity: linear = 200 mm/s -> linear = minLinVel mm/s
	if err := consecutiveVelocityTest(b, odometry, r3.Vector{Y: 200.0}, r3.Vector{Y: minLinVel}); err != nil {
		storeErr := fmt.Errorf("error with consecutive SetVelocity calls, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// MoveStraight: distance = 100 mm, speed = 50 mm/sec
	if err := moveStraightTest(b, odometry, 100, 50); err != nil {
		storeErr := fmt.Errorf("error moving straight for 100 mm at 50 mm/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// MoveStraight: distance = -100 mm, speed = 250 mm/sec
	if err := moveStraightTest(b, odometry, -100, 250); err != nil {
		storeErr := fmt.Errorf("error moving straight for -100 mm at 250 mm/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// MoveStraight: distance = 1000 mm, speed = -50 mm/sec
	if err := moveStraightTest(b, odometry, 1000, -50); err != nil {
		storeErr := fmt.Errorf("error moving straight for 1000 mm at -50 mm/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// MoveStraight: distance = -1000 mm, speed = -250 mm/sec
	if err := moveStraightTest(b, odometry, -1000, -250); err != nil {
		storeErr := fmt.Errorf("error moving straight for -1000 mm at -250 mm/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// Spin: distance = 30 deg, speed = 15 deg/sec
	if err := spinTest(b, odometry, 30, 15, false); err != nil {
		storeErr := fmt.Errorf("error spinning for 30 deg at 15 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// Spin: distance = 30 deg, speed = -60 deg/sec
	if err := spinTest(b, odometry, 30, -60, false); err != nil {
		storeErr := fmt.Errorf("error spinning for 30 deg at -60 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// Spin: distance = -360 deg, speed = 15 deg/sec
	if err := spinTest(b, odometry, -360, 15, true); err != nil {
		storeErr := fmt.Errorf("error spinning for -360 deg at 15 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// Spin: distance = -360 deg, speed = -90 deg/sec
	if err := spinTest(b, odometry, -360, -90, true); err != nil {
		storeErr := fmt.Errorf("error spinning for -360 deg at -90 deg/sec, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetPower: power = 10% / Stop
	if err := baseSetPowerTest(b, odometry, 0.1); err != nil {
		storeErr := fmt.Errorf("error setting power, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetPower: power = 90% / Stop
	if err := baseSetPowerTest(b, odometry, 0.9); err != nil {
		storeErr := fmt.Errorf("error setting power, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}

	// SetPower: power = -50% / Stop
	if err := baseSetPowerTest(b, odometry, -0.5); err != nil {
		storeErr := fmt.Errorf("error setting power, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", b.Name().ShortName(), storeErr))
	}
}

func runMotorTests(m motor.Motor, odometry movementsensor.MovementSensor) {
	// GoFor: distance = 1 rev, speed = 10 rpm
	if err := goForTest(m, odometry, 10, 1); err != nil {
		storeErr := fmt.Errorf("error going for 1 rev at 10 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// GoFor: distance = -5 rev, speed = 50 rpm
	if err := goForTest(m, odometry, 50, -5); err != nil {
		storeErr := fmt.Errorf("error going for -5 rev at 50 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// GoFor: distance = 5 rev, speed = -10 rpm
	if err := goForTest(m, odometry, -10, 5); err != nil {
		storeErr := fmt.Errorf("error going for 5 rev at -10 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// GoFor: distance = -5 rev, speed = -50 rpm
	if err := goForTest(m, odometry, -50, -5); err != nil {
		storeErr := fmt.Errorf("error going for -5 rev at -50 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// GoTo: position = -5, speed = 10 rpm, ResetZeroPosition: offset = -2
	if err := goToTest(m, odometry, 10, -5); err != nil {
		storeErr := fmt.Errorf("error going to position -5 at 10 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// GoTo: position = 0, speed = 50 rpm
	if err := goToTest(m, odometry, 50, 0); err != nil {
		storeErr := fmt.Errorf("error going to position 0 at 50 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// SetRPM: speed = 10 rpm
	if err := setRPMTest(m, odometry, 10); err != nil {
		storeErr := fmt.Errorf("error setting rpm at 10 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// SetRPM: speed = -50 rpm
	if err := setRPMTest(m, odometry, -50); err != nil {
		storeErr := fmt.Errorf("error setting rpm at -50 rpm, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// SetRPM: rpm = 30 -> rpm = 60
	if err := consecutiveRPMTest(m, odometry, 30, 60); err != nil {
		storeErr := fmt.Errorf("error with consecutive SetRPM calls, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// SetPower: power = 10% / Stop
	if err := motorSetPowerTest(m, 0.1); err != nil {
		storeErr := fmt.Errorf("error setting power, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}

	// SetPower: power = -90% / Stop
	if err := motorSetPowerTest(m, -0.9); err != nil {
		storeErr := fmt.Errorf("error setting power, err = %v", err)
		failedTests = append(failedTests, fmt.Sprintf("%v: %v", m.Name().ShortName(), storeErr))
	}
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

func setVelocityTest(b base.Base, odometry movementsensor.MovementSensor, linear, angular r3.Vector) error {
	if err := b.SetVelocity(context.Background(), linear, angular, nil); err != nil {
		return err
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	linEst, angEst := sampleBothVel(odometry, linear.Y, angular.Z)

	if err := b.Stop(context.Background(), nil); err != nil {
		return err
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
		return fmt.Errorf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", linEst, angEst, linear.Y, angular.Z)
	}
	return nil
}

func consecutiveVelocityTest(b base.Base, odometry movementsensor.MovementSensor, linear1, linear2 r3.Vector) error {
	// SetVelocity with linear1
	if err := b.SetVelocity(context.Background(), linear1, r3.Vector{}, nil); err != nil {
		return err
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	linEst, angEst := sampleBothVel(odometry, linear1.Y, 0.0)

	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear1.Y, linEst, linear1.Y*0.5) || !rdkutils.Float64AlmostEqual(0.0, angEst, 15.0) {
		return fmt.Errorf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", linEst, angEst, linear1.Y, 0.0)
	}

	// SetVelocity with linear 2
	if err := b.SetVelocity(context.Background(), linear2, r3.Vector{}, nil); err != nil {
		return err
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	linEst, angEst = sampleBothVel(odometry, linear2.Y, 0.0)

	// verify average speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(linear2.Y, linEst, linear2.Y*0.5) || !rdkutils.Float64AlmostEqual(0.0, angEst, 15.0) {
		return fmt.Errorf("measured velocity (linear: %v, angular: %v) did not equal requested velocity (linear: %v, angular: %v)", linEst, angEst, linear2.Y, 0.0)
	}

	return b.Stop(context.Background(), nil)
}

func moveStraightTest(b base.Base, odometry movementsensor.MovementSensor, distance, speed float64) error {
	dir := sign(distance * speed)

	startPos, _, err := odometry.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	var speedEst float64
	go func() {
		linVel := sampleLinearVel(odometry, math.Abs(speed)*dir, math.Abs(distance/speed))
		speedEst = linVel
	}()

	if err := b.MoveStraight(context.Background(), int(distance), speed, nil); err != nil {
		return err
	}
	endPos, _, err := odometry.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := startPos.GreatCircleDistance(endPos) * 1000000.0

	// wait for sampleLinearVel to return
	for {
		if speedEst != 0 {
			break
		}
	}

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist*dir, math.Abs(distance)*dir, math.Abs(distance*0.3)) {
		return fmt.Errorf("measured distance %v did not equal requested distance %v", totalDist*dir, distance)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(speedEst, math.Abs(speed)*dir, math.Abs(speed*0.5)) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", speedEst, math.Abs(speed)*dir)
	}

	return nil
}

func spinTest(b base.Base, odometry movementsensor.MovementSensor, distance, speed float64, testSpeed bool) error {
	dir := sign(distance * speed)

	var speedEst float64
	go func() {
		angVel := sampleAngularVel(odometry, math.Abs(speed)*dir, math.Abs(distance/speed))
		speedEst = angVel
	}()

	startPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return err
	}
	startTime := time.Now()
	if err := b.Spin(context.Background(), distance, speed, nil); err != nil {
		return err
	}
	endTime := time.Now()
	endPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := distBetweenAngles(endPos.OrientationVectorDegrees().Theta, startPos.OrientationVectorDegrees().Theta, (math.Abs(distance) * dir))

	// wait for sampleAngularVelocity to return
	for {
		if speedEst != 0 {
			break
		}
	}

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, math.Abs(distance)*dir, math.Abs(distance*0.3)) {
		return fmt.Errorf("measured distance %v did not equal requested distance %v", totalDist, distance)
	}

	if testSpeed {
		// verify speed is approximately requested speed
		if !rdkutils.Float64AlmostEqual(speedEst, math.Abs(speed)*dir, math.Abs(speed)*0.5) {
			return fmt.Errorf("measured speed %v did not equal requested speed %v", speedEst, speed)
		}
	} else {
		if float64(endTime.Sub(startTime))*1e-9 > 5 {
			return fmt.Errorf("spin call took longer than expected, actual time = %v, max allowed time = %v", float64(endTime.Sub(startTime))*1e-9, math.Abs(distance/speed*5))
		}
	}

	return nil
}

func baseSetPowerTest(b base.Base, odometry movementsensor.MovementSensor, power float64) error {
	// if power is negative, just test linear power
	angPwr := 0.0
	if power > 0 {
		angPwr = 1 - power
	}
	if err := b.SetPower(context.Background(), r3.Vector{Y: power}, r3.Vector{Z: angPwr}, nil); err != nil {
		return err
	}

	// wait for base to start moving
	time.Sleep(1 * time.Second)
	powered, err := b.IsMoving(context.Background())
	if err != nil {
		return err
	}

	if !powered {
		return fmt.Errorf("base is not powered (linear = %v, angular = %v)", power, angPwr)
	}

	linVel, err := odometry.LinearVelocity(context.Background(), nil)
	if err != nil {
		return err
	}
	angVel, err := odometry.AngularVelocity(context.Background(), nil)
	if err != nil {
		return err
	}

	if power < 0 {
		if linVel.Y > 0 {
			return fmt.Errorf("base should be moving in the negative direction (linear = %v, angular = %v)", linVel.Y, angVel.Z)
		}
	} else {
		if linVel.Y < 0 || angVel.Z < 0 {
			return fmt.Errorf("base should be moving in the positive direction (linear = %v, angular = %v)", linVel.Y, angVel.Z)
		}
	}

	if err := b.Stop(context.Background(), nil); err != nil {
		return err
	}
	powered, err = b.IsMoving(context.Background())
	if err != nil {
		return err
	}

	if powered {
		return fmt.Errorf("base did not stop with linear = %v, angular = %v", power, 1-power)
	}

	return nil
}

func goForTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm, revolutions float64) error {
	dir := sign(rpm * revolutions)

	goalLinVel := rpm * wheelCircumference / 60

	var speedEst float64
	go func() {
		linVel := sampleLinearVel(odometry, math.Abs(goalLinVel)*dir, math.Abs(revolutions/rpm*60))
		speedEst = linVel
	}()

	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}
	if err := m.GoFor(context.Background(), rpm, revolutions, nil); err != nil {
		return err
	}

	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := endPos - startPos

	// wait for sampleLinearVel to return
	for {
		if speedEst != 0 {
			break
		}
	}

	rpmEst := speedEst / wheelCircumference * 60
	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, math.Abs(revolutions)*dir, math.Abs(revolutions*0.3)) {
		return fmt.Errorf("measured revolutions %v did not equal requested revolutions %v", totalDist, revolutions)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, math.Abs(rpm)*dir, math.Abs(rpm*0.5)) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", rpmEst, rpm)
	}
	return nil
}

func goToTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm, position float64) error {
	var speedEst float64

	if err := m.ResetZeroPosition(context.Background(), -2, nil); err != nil {
		return err
	}

	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	dir := sign((position - startPos) * rpm)
	goalLinVel := rpm * wheelCircumference / 60

	go func() {
		linVel := sampleLinearVel(odometry, math.Abs(goalLinVel)*dir, math.Abs(position-startPos/rpm*60))
		speedEst = linVel
	}()

	if err := m.GoTo(context.Background(), rpm, position, nil); err != nil {
		return err
	}
	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	// wait for sampleLinearVel to return
	for {
		if speedEst != 0 {
			break
		}
	}
	rpmEst := speedEst / wheelCircumference * 60

	// verify start position is 2 after ResetZeroPosition
	if startPos != 2 {
		return fmt.Errorf("startPos = %v when it should be 2", startPos)
	}

	// verify end position is approximately requested end position
	if !rdkutils.Float64AlmostEqual(endPos, position, 1) {
		return fmt.Errorf("measured end position %v did not equal requested end position %v", endPos, position)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(math.Abs(rpmEst), math.Abs(rpm), math.Abs(rpm*0.5)) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", math.Abs(rpmEst), math.Abs(rpm))
	}
	return nil
}

func setRPMTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm float64) error {
	if err := m.SetRPM(context.Background(), rpm, nil); err != nil {
		return err
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	goalLinVel := rpm * wheelCircumference / 60
	speedEst := sampleLinearVel(odometry, math.Abs(goalLinVel)*sign(rpm), 5)

	if err := m.Stop(context.Background(), nil); err != nil {
		return err
	}

	rpmEst := speedEst / wheelCircumference * 60

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, rpm, math.Abs(rpm)*0.5) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", rpmEst, rpm)
	}
	return nil
}

func consecutiveRPMTest(m motor.Motor, odometry movementsensor.MovementSensor, rpm1, rpm2 float64) error {
	// SetRPM with rpm1
	if err := m.SetRPM(context.Background(), rpm1, nil); err != nil {
		return err
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	goalLinVel := rpm1 * wheelCircumference / 60
	speedEst := sampleLinearVel(odometry, math.Abs(goalLinVel)*sign(rpm1), 5)

	rpmEst := speedEst / wheelCircumference * 60

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, rpm1, math.Abs(rpm1)*0.5) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", rpmEst, rpm1)
	}

	// SetRPM with rpm2
	if err := m.SetRPM(context.Background(), rpm2, nil); err != nil {
		return err
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	goalLinVel = rpm2 * wheelCircumference / 60
	speedEst = sampleLinearVel(odometry, math.Abs(goalLinVel)*sign(rpm2), 5)

	rpmEst = speedEst / wheelCircumference * 60

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(rpmEst, rpm2, math.Abs(rpm2)*0.5) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", rpmEst, rpm2)
	}

	return m.Stop(context.Background(), nil)
}

func motorSetPowerTest(m motor.Motor, power float64) error {
	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	if err := m.SetPower(context.Background(), power, nil); err != nil {
		return err
	}

	// wait for motor to start moving
	time.Sleep(2 * time.Second)
	powered, powerPct, err := m.IsPowered(context.Background(), nil)
	if err != nil {
		return err
	}

	if !powered {
		return fmt.Errorf("motor is not powered (power = %v)", powerPct)
	}

	if !rdkutils.Float64AlmostEqual(powerPct, power, math.Abs(power)*0.3) {
		return fmt.Errorf("measured power %v does not match requested power %v", powerPct, power)
	}

	if err := m.Stop(context.Background(), nil); err != nil {
		return err
	}
	powered, err = m.IsMoving(context.Background())
	if err != nil {
		return err
	}

	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	if sign(endPos-startPos) != sign(power) {
		return fmt.Errorf("motor dir = %v, requested motor dir = %v", sign(endPos-startPos), sign(power))
	}

	if powered {
		return fmt.Errorf("motor did not stop with power = %v", power)
	}

	return nil
}

func runGridTest(b base.Base, odometry movementsensor.MovementSensor) {
	gridPath := []string{"long-straight", "left", "short-straight", "left", "long-straight", "right", "short-straight", "right", "long-straight", "left", "short-straight", "left", "long-straight"}
	odometry.DoCommand(context.Background(), map[string]interface{}{"reset": true})

	rmsErrorSum := 0.0
	rmsNumSamples := 0.0

	desVel := 100.0
	desDist := 0.0
	desAng := 0.0
	desAngVel := 30.0

	posLat, posLng := 0.0, 0.0
	lastAng := 0.0

	for _, s := range gridPath {
		switch s {
		case "long-straight":
			desDist = 1500.0

			if lastAng == 0 || lastAng == 360 {
				posLat += desDist
			} else if lastAng == 180 {
				posLat -= desDist
			} else if lastAng == -90 {
				posLng += desDist
			} else if lastAng == 90 {
				posLng -= desDist
			}

			var lat, lng []float64
			go func() {
				lat, lng = samplePosition(odometry, lat, lng, true)
			}()

			err := b.MoveStraight(context.Background(), int(desDist), desVel, nil)
			if err != nil {
				logger.Error(err)
				return
			}

			for {
				if lat != nil {
					break
				}
			}

			numSamples := float64(len(lat))
			rmsNumSamples += numSamples
			totalDist := desDist / 1000
			var diff float64
			for i := 0.0; i < numSamples; i++ {
				predictedPoint := geo.NewPoint(((totalDist / numSamples) * i), posLng/1000)
				actualPoint := geo.NewPoint(lat[int(i)], lng[int(i)])
				// diff = (predicted - actual)^2
				diff = math.Pow(actualPoint.GreatCircleDistance(predictedPoint), 2)
				rmsErrorSum += diff
			}

		case "short-straight":
			desDist = 500.0

			if lastAng == 0 || lastAng == 360 {
				posLat += desDist
			} else if lastAng == 180 {
				posLat -= desDist
			} else if lastAng == -90 {
				posLng += desDist
			} else if lastAng == 90 {
				posLng -= desDist
			}

			var lat, lng []float64
			go func() {
				lat, lng = samplePosition(odometry, lat, lng, false)
			}()

			err := b.MoveStraight(context.Background(), int(desDist), desVel, nil)
			if err != nil {
				logger.Error(err)
				return
			}

			for {
				if lng != nil {
					break
				}
			}

			numSamples := float64(len(lng))
			rmsNumSamples += numSamples
			totalDist := desDist / 1000
			var diff float64
			for i := 0.0; i < numSamples; i++ {
				predictedPoint := geo.NewPoint(posLat/1000, ((totalDist / numSamples) * i))
				actualPoint := geo.NewPoint(lat[int(i)], lng[int(i)])
				// diff = (predicted - actual)^2
				diff = math.Pow(actualPoint.GreatCircleDistance(predictedPoint), 2)
				rmsErrorSum += diff
			}

		case "left":
			desAng = 90
			lastAng += desAng
			if lastAng > 360 {
				lastAng -= 360
			}

			err := b.Spin(context.Background(), desAng, desAngVel, nil)
			if err != nil {
				logger.Error(err)
				return
			}

		case "right":
			desAng = -90
			lastAng += desAng
			if lastAng > 360 {
				lastAng -= 360
			}

			err := b.Spin(context.Background(), desAng, desAngVel, nil)
			if err != nil {
				logger.Error(err)
				return
			}
		}
	}

	rmsErr := math.Sqrt(rmsErrorSum / rmsNumSamples)

	if rmsErr > 150 {
		storeErr := fmt.Errorf("rms error %v is higher than the minimum allowed error %v", rmsErr, 150)
		failedTests = append(failedTests, fmt.Sprintf("grid test: %v", storeErr))
	}
}

func samplePosition(odometry movementsensor.MovementSensor, lat, lng []float64, isLat bool) ([]float64, []float64) {
	lastLat, lastLng := 0.0, 0.0
	for {
		linVel, err := odometry.LinearVelocity(context.Background(), nil)
		if err != nil {
			logger.Error(err)
			return nil, nil
		}
		if linVel.Y != 0 {
			break
		}
	}
	for {
		pos, _, err := odometry.Position(context.Background(), posExtra)
		if err != nil {
			logger.Error(err)
		}

		lat = append(lat, pos.Lat())
		lng = append(lng, pos.Lng())

		if isLat {
			if pos.Lat() == lastLat {
				return lat, lng
			}
			lastLat = pos.Lat()
		} else {
			if pos.Lng() == lastLng {
				return lat, lng
			}
			lastLng = pos.Lng()
		}
	}
}

func sign(num float64) float64 {
	if num < 0.0 {
		return -1.0
	}
	return 1.0
}

func distBetweenAngles(endDeg, startDeg, originalDist float64) float64 {
	totalDeg := endDeg - startDeg
	if sign(totalDeg) != sign(originalDist) && !rdkutils.Float64AlmostEqual(0, totalDeg, 10) {
		totalDeg += sign(originalDist) * 360
	}
	turns := (int(originalDist) / 360.0) % 360
	return totalDeg + float64(turns*360)
}

// timeEst must be absolute value
func sampleLinearVel(odometry movementsensor.MovementSensor, goalVel, timeEst float64) float64 {
	maxSpeed := -0.000000001
	start := time.Now()
	for {
		linVel, err := odometry.LinearVelocity(context.Background(), nil)
		if err != nil {
			logger.Error(err)
			return -1
		}
		if sign(goalVel) < 0 {
			if linVel.Y < maxSpeed {
				maxSpeed = linVel.Y
			}
		} else {
			if linVel.Y > maxSpeed {
				maxSpeed = linVel.Y
			}
		}
		if rdkutils.Float64AlmostEqual(goalVel, linVel.Y*1000, math.Abs(goalVel)*0.5) {
			return linVel.Y * 1000.0
		}
		timeElapsed := float64(time.Since(start)) * 1e-9
		if float64(timeElapsed) > timeEst*2 {
			if maxSpeed == 0 {
				return -1
			}
			return maxSpeed * 1000.0

		}
	}
}

func sampleAngularVel(odometry movementsensor.MovementSensor, goalVel, timeEst float64) float64 {
	maxSpeed := 0.0
	start := time.Now()
	for {
		angVel, err := odometry.AngularVelocity(context.Background(), nil)
		if err != nil {
			logger.Error(err)
			return -1
		}
		if math.Abs(angVel.Z) > math.Abs(maxSpeed) {
			maxSpeed = angVel.Z
		}
		if rdkutils.Float64AlmostEqual(goalVel, angVel.Z, math.Abs(goalVel)*0.5) {
			return angVel.Z
		}
		timeElapsed := float64(time.Since(start)) * 1e-9
		if float64(timeElapsed) > timeEst*2 {
			return maxSpeed

		}
	}
}

func sampleBothVel(odometry movementsensor.MovementSensor, goalLinVel, goalAngVel float64) (float64, float64) {
	start := time.Now()
	for {
		linVel, err := odometry.LinearVelocity(context.Background(), nil)
		if err != nil {
			logger.Error(err)
			return -1, -1
		}
		angVel, err := odometry.AngularVelocity(context.Background(), nil)
		if err != nil {
			logger.Error(err)
			return -1, -1
		}

		linErr, angErr := 50.0, 15.0
		if goalLinVel != 0 {
			linErr = math.Abs(goalLinVel) * 0.5
		}
		if goalAngVel != 0 {
			angErr = math.Abs(goalAngVel) * 0.5
		}

		if rdkutils.Float64AlmostEqual(goalLinVel, linVel.Y, linErr) &&
			rdkutils.Float64AlmostEqual(goalAngVel, angVel.Z, angErr) {
			return linVel.Y * 1000, angVel.Z
		}

		timeElapsed := float64(time.Since(start)) * 1e-9
		if float64(timeElapsed) > 10 {
			return linVel.Y * 1000, angVel.Z
		}
	}
}

func sendSlackMessage(msg string) {
	data := []byte("{'text': '" + msg + "'}")
	body := bytes.NewReader(data)

	req, err := http.NewRequest("POST", "<WEBHOOK>", body)
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
