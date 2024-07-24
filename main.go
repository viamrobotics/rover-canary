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
	"go.viam.com/rdk/components/powersensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"
)

var (
	logger           = logging.NewLogger("client")
	ticksPerRotation = 1992.0
)

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

	// controlled motor tests
	logger.Info("Starting controlled motor tests...")
	if err := runMotorTests(rightMotor, rightEncoder); err != nil {
		logger.Error(err)
		return
	}

	// encoded motor tests
	logger.Info("Starting encoded motor tests...")
	if err := runMotorTests(leftMotor, leftEncoder); err != nil {
		logger.Error(err)
		return
	}

	// single encoder tests
	logger.Info("Starting encoder tests...")
	if err := runEncoderTests(leftMotor, leftEncoder); err != nil {
		logger.Error(err)
		return
	}

	// power sensor tests
	logger.Info("Starting power sensor tests...")
	if err := runPowerSensorTests(powerSensor); err != nil {
		logger.Error(err)
		return
	}

	// movement sensor tests
	logger.Info("Starting movement sensor tests...")
	if err := runMovementSensorTests(movementSensor); err != nil {
		logger.Error(err)
		return
	}

	logger.Info("All tests passed successfully!")
}

func runBaseTests(b base.Base, odometry movementsensor.MovementSensor) error {
	// SetVelocity: linear = 50 mm/s, angular = 0 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: 50.0}, r3.Vector{}); err != nil {
		return fmt.Errorf("error setting velocity to linear = 50 mm/s and anguar = 0 deg/sec, err = %v", err)
	}

	// SetVelocity: linear = 250 mm/s, angular = 0 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{Y: -250.0}, r3.Vector{}); err != nil {
		return fmt.Errorf("error setting velocity to linear = -250 mm/s and anguar = 0 deg/sec, err = %v", err)
	}

	// SetVelocity: linear = 0 mm/s, angular = 10 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{}, r3.Vector{Z: -15.0}); err != nil {
		return fmt.Errorf("error setting velocity to linear = 0 mm/s and anguar = -15 deg/sec, err = %v", err)
	}

	// SetVelocity: linear = 0 mm/s, angular = 90 deg/sec
	if err := setVelocityTest(b, odometry, r3.Vector{}, r3.Vector{Z: 90.0}); err != nil {
		return fmt.Errorf("error setting velocity to linear = 0 mm/s and anguar = 90 deg/sec, err = %v", err)
	}

	// MoveStraight: distance = 100 mm, speed = 50 mm/sec
	if err := moveStraightTest(b, odometry, 100, 50); err != nil {
		return fmt.Errorf("error moving straight for 100 mm at 50 mm/sec, err = %v", err)
	}

	// MoveStraight: distance = -100 mm, speed = 300 mm/sec
	if err := moveStraightTest(b, odometry, -100, 300); err != nil {
		return fmt.Errorf("error moving straight for -100 mm at 300 mm/sec, err = %v", err)
	}

	// MoveStraight: distance = 1000 mm, speed = -50 mm/sec
	if err := moveStraightTest(b, odometry, 1000, -50); err != nil {
		return fmt.Errorf("error moving straight for 1000 mm at -50 mm/sec, err = %v", err)
	}

	// MoveStraight: distance = -1000 mm, speed = -300 mm/sec
	if err := moveStraightTest(b, odometry, -1000, -300); err != nil {
		return fmt.Errorf("error moving straight for -1000 mm at -300 mm/sec, err = %v", err)
	}

	// Spin: distance = 15 deg, speed = 15 deg/sec
	if err := spinTest(b, odometry, 15, 15, false); err != nil {
		return fmt.Errorf("error spinning for 15 deg at 15 deg/sec, err = %v", err)
	}

	// Spin: distance = 15 deg, speed = -90 deg/sec
	if err := spinTest(b, odometry, 15, -90, false); err != nil {
		return fmt.Errorf("error spinning for 15 deg at -90 deg/sec, err = %v", err)
	}

	// Spin: distance = -540 deg, speed = 15 deg/sec
	if err := spinTest(b, odometry, -360, 15, true); err != nil {
		return fmt.Errorf("error spinning for -360 deg at 15 deg/sec, err = %v", err)
	}

	// Spin: distance = -540 deg, speed = -90 deg/sec
	if err := spinTest(b, odometry, -540, -90, true); err != nil {
		return fmt.Errorf("error spinning for -360 deg at -90 deg/sec, err = %v", err)
	}

	// SetPower: power = 10% / Stop
	if err := baseSetPowerTest(b, 0.1); err != nil {
		return fmt.Errorf("error setting power, err = %v", err)
	}

	// SetPower: power = 90% / Stop
	if err := baseSetPowerTest(b, 0.9); err != nil {
		return fmt.Errorf("error setting power, err = %v", err)
	}

	return nil
}

func runMotorTests(m motor.Motor, enc encoder.Encoder) error {
	logger.Error("GO FOR")
	// GoFor: distance = 1 rev, speed = 10 rpm
	if err := goForTest(m, enc, 10, 1); err != nil {
		return fmt.Errorf("error going for 1 rev at 10 rpm, err = %v", err)
	}

	// GoFor: distance = -1 rev, speed = 50 rpm
	if err := goForTest(m, enc, 50, -5); err != nil {
		return fmt.Errorf("error going for -5 rev at 50 rpm, err = %v", err)
	}

	// GoFor: distance = 5 rev, speed = -10 rpm
	if err := goForTest(m, enc, -10, 5); err != nil {
		return fmt.Errorf("error going for 5 rev at -10 rpm, err = %v", err)
	}

	// GoFor: distance = -5 rev, speed = -50 rpm
	if err := goForTest(m, enc, -50, -5); err != nil {
		return fmt.Errorf("error going for -5 rev at -50 rpm, err = %v", err)
	}

	logger.Error("GO TO")
	// GoTo: position = -5, speed = 10 rpm
	if err := goToTest(m, enc, 10, -5); err != nil {
		return fmt.Errorf("error going to position -5 at 10 rpm, err = %v", err)
	}

	// GoTo: position = 0, speed = 50 rpm
	if err := goToTest(m, enc, 50, 0); err != nil {
		return fmt.Errorf("error going to position 0 at 50 rpm, err = %v", err)
	}

	logger.Error("SET RPM")
	// SetRPM: speed = 10 rpm
	if err := setRPMTest(m, enc, 10); err != nil {
		return fmt.Errorf("error setting rpm at 10 rpm, err = %v", err)
	}

	// SetRPM: speed = -50 rpm
	if err := setRPMTest(m, enc, -50); err != nil {
		return fmt.Errorf("error setting rpm at -50 rpm, err = %v", err)
	}

	logger.Error("SET POWER")
	// SetPower: power = 10% / Stop
	if err := motorSetPowerTest(m, 0.1); err != nil {
		return fmt.Errorf("error setting power, err = %v", err)
	}

	// SetPower: power = 90% / Stop
	if err := motorSetPowerTest(m, 0.9); err != nil {
		return fmt.Errorf("error setting power, err = %v", err)
	}

	return nil
}

func runEncoderTests(m motor.Motor, enc encoder.Encoder) error {
	// motor position
	pos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	// encoder positon
	ticks, _, err := enc.Position(context.Background(), 0, nil)
	if err != nil {
		return err
	}

	// verify ticks is approximately motor position
	if !rdkutils.Float64AlmostEqual(ticks, pos*ticksPerRotation, 10) {
		return fmt.Errorf("measured encoder position %v did not equal motor position %v", ticks, pos*ticksPerRotation)
	}

	// reset position
	if err := enc.ResetPosition(context.Background(), nil); err != nil {
		return err
	}

	// motor position
	pos, err = m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	// encoder positon
	ticks, _, err = enc.Position(context.Background(), 0, nil)
	if err != nil {
		return err
	}

	// verify ticks is 0
	if ticks != 0 {
		return fmt.Errorf("ticks is not equal to 0 after reset, ticks = %v", ticks)
	}

	// verify ticks and motor position are zero
	if ticks != pos*ticksPerRotation || ticks != 0.0 {
		return fmt.Errorf("measured encoder position %v did not equal motor position %v", ticks, pos*ticksPerRotation)
	}

	return nil
}

func runPowerSensorTests(ps powersensor.PowerSensor) error {
	volts, _, err := ps.Voltage(context.Background(), nil)
	if err != nil {
		return err
	}

	current, _, err := ps.Current(context.Background(), nil)
	if err != nil {
		return err
	}

	power, err := ps.Power(context.Background(), nil)
	if err != nil {
		return err
	}

	// verify voltage is ~15.2
	if !rdkutils.Float64AlmostEqual(volts, 15.2, 0.5) {
		return fmt.Errorf("voltage does not equal 15.2, voltage = %v", err)
	}

	// verify current is ~0.29
	if !rdkutils.Float64AlmostEqual(current, 0.29, 0.1) {
		return fmt.Errorf("voltage does not equal 15.2, voltage = %v", err)
	}

	// verify power is ~4.4
	if !rdkutils.Float64AlmostEqual(power, 4.4, 0.25) {
		return fmt.Errorf("voltage does not equal 15.2, voltage = %v", err)
	}

	return nil
}

func runMovementSensorTests(ms movementsensor.MovementSensor) error {
	linearAccel, err := ms.LinearAcceleration(context.Background(), nil)
	if err != nil {
		return err
	}

	// verify linear acceleration is ~9.81
	if !rdkutils.Float64AlmostEqual(linearAccel.Z, 9.81, 1) {
		return fmt.Errorf("linear acceleration is not ~9.81, linear acceleration = %v", linearAccel)
	}
	return nil
}

func setVelocityTest(b base.Base, odometry movementsensor.MovementSensor, linear, angular r3.Vector) error {
	if err := b.SetVelocity(context.Background(), linear, angular, nil); err != nil {
		return err
	}

	// let the base get up to speed for 5 seconds
	time.Sleep(5 * time.Second)

	linearSum, angularSum := 0.0, 0.0
	// sample linear and angular velocities for 10 seconds
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
	if err := b.Stop(context.Background(), nil); err != nil {
		return err
	}

	avgLinear := linearSum / 10.0
	avgAngular := angularSum / 10.0

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

func moveStraightTest(b base.Base, odometry movementsensor.MovementSensor, distance, speed float64) error {
	dir := sign(distance * speed)

	var totalTime time.Duration
	go func() error {
		time, err := baseVelocityEstimate(b)
		totalTime = time
		return err
	}()

	startPos, _, err := odometry.Position(context.Background(), nil)
	if err != nil {
		return err
	}
	if err := b.MoveStraight(context.Background(), int(distance), speed, nil); err != nil {
		return err
	}
	endPos, _, err := odometry.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := startPos.GreatCircleDistance(endPos) * 1000000.0
	speedEst := totalDist / (float64(totalTime) * 1e-9)

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist*dir, math.Abs(distance)*dir, math.Abs(distance*0.3)) {
		return fmt.Errorf("measured distance %v did not equal requested distance %v", totalDist*dir, distance)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(speedEst, math.Abs(speed), math.Abs(speed*0.5)) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", speedEst, math.Abs(speed))
	}

	return nil
}

func spinTest(b base.Base, odometry movementsensor.MovementSensor, distance, speed float64, testSpeed bool) error {
	dir := sign(distance * speed)

	var totalTime time.Duration
	go func() error {
		time, err := baseVelocityEstimate(b)
		totalTime = time
		return err
	}()

	startPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return err
	}
	if err := b.Spin(context.Background(), distance, speed, nil); err != nil {
		return err
	}
	endPos, err := odometry.Orientation(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := distBetweenAngles(endPos.OrientationVectorDegrees().Theta, startPos.OrientationVectorDegrees().Theta, int(math.Abs(distance)*dir))
	angVel := totalDist / (float64(totalTime) * 1e-9)

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, math.Abs(distance)*dir, math.Abs(distance*0.3)) {
		return fmt.Errorf("measured distance %v did not equal requested distance %v", totalDist, distance)
	}

	if testSpeed {
		// verify speed is approximately requested speed
		if !rdkutils.Float64AlmostEqual(angVel, math.Abs(speed)*dir, math.Abs(speed)*0.5) {
			return fmt.Errorf("measured speed %v did not equal requested speed %v", angVel, speed)
		}
	}

	return nil
}

func baseSetPowerTest(b base.Base, power float64) error {
	if err := b.SetPower(context.Background(), r3.Vector{Y: power}, r3.Vector{Z: 1 - power}, nil); err != nil {
		return err
	}

	time.Sleep(1 * time.Second)
	powered, err := b.IsMoving(context.Background())
	if err != nil {
		return err
	}

	if !powered {
		return fmt.Errorf("base is not powered (linear = %v, angular = %v)", power, 1-power)
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

func goForTest(m motor.Motor, enc encoder.Encoder, rpm, revolutions float64) error {
	if err := enc.ResetPosition(context.Background(), nil); err != nil {
		return err
	}
	dir := sign(rpm * revolutions)

	var totalTime time.Duration
	go func() error {
		time, err := motorVelocityEstimate(m)
		totalTime = time
		return err
	}()

	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}
	if err := m.GoFor(context.Background(), rpm, revolutions, nil); err != nil {
		return err
	}
	time.Sleep(1 * time.Second)
	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	totalDist := endPos - startPos
	speedEst := totalDist / (float64(totalTime) * 1e-9) * 60

	// verify distance is approximately requested distance
	if !rdkutils.Float64AlmostEqual(totalDist, math.Abs(revolutions)*dir, math.Abs(revolutions*0.3)) {
		return fmt.Errorf("measured revolutions %v did not equal requested revolutions %v", totalDist, revolutions)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(speedEst, math.Abs(rpm)*dir, math.Abs(rpm*0.5)) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", speedEst, rpm)
	}
	return nil
}

func goToTest(m motor.Motor, enc encoder.Encoder, rpm, position float64) error {
	if err := enc.ResetPosition(context.Background(), nil); err != nil {
		return err
	}
	var totalTime time.Duration
	go func() error {
		time, err := motorVelocityEstimate(m)
		totalTime = time
		return err
	}()

	startPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	if err := m.GoTo(context.Background(), rpm, position, nil); err != nil {
		return err
	}
	endPos, err := m.Position(context.Background(), nil)
	if err != nil {
		return err
	}

	speedEst := (endPos - startPos) / (float64(totalTime) * 1e-9) * 60

	// verify end position is approximately requested end position
	if !rdkutils.Float64AlmostEqual(endPos, position, 1) {
		return fmt.Errorf("measured end position %v did not equal requested end position %v", endPos, position)
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(math.Abs(speedEst), math.Abs(rpm), math.Abs(rpm*0.3)) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", math.Abs(speedEst), math.Abs(rpm))
	}
	return nil
}

func setRPMTest(m motor.Motor, enc encoder.Encoder, rpm float64) error {
	if err := m.SetRPM(context.Background(), rpm, nil); err != nil {
		return err
	}

	// allow motor to get up to speed
	time.Sleep(5 * time.Second)

	sampledSpeed, err := sampleMotorRPM(enc)
	if err != nil {
		return err
	}
	if err := m.Stop(context.Background(), nil); err != nil {
		return err
	}

	// verify speed is approximately requested speed
	if !rdkutils.Float64AlmostEqual(sampledSpeed, rpm, math.Abs(rpm)*0.3) {
		return fmt.Errorf("measured speed %v did not equal requested speed %v", sampledSpeed, rpm)
	}
	return nil
}

func motorSetPowerTest(m motor.Motor, power float64) error {
	if err := m.SetPower(context.Background(), power, nil); err != nil {
		return err
	}

	time.Sleep(1 * time.Second)
	powered, powerPct, err := m.IsPowered(context.Background(), nil)
	if err != nil {
		return err
	}

	if !powered {
		return fmt.Errorf("motor is not powered (power = %v)", power)
	}

	if !rdkutils.Float64AlmostEqual(powerPct, power, power*0.3) {
		return fmt.Errorf("measured power %v does not match requested power %v", powerPct, power)
	}

	if err := m.Stop(context.Background(), nil); err != nil {
		return err
	}
	powered, err = m.IsMoving(context.Background())
	if err != nil {
		return err
	}

	if powered {
		return fmt.Errorf("motor did not stop with power = %v", power)
	}

	return nil
}

func sign(num float64) float64 {
	if num < 0.0 {
		return -1.0
	}
	return 1.0
}

func sampleMotorRPM(enc encoder.Encoder) (float64, error) {
	startPos, _, err := enc.Position(context.Background(), 0, nil)
	if err != nil {
		return 0, err
	}
	time.Sleep(4 * time.Second)
	endPos, _, err := enc.Position(context.Background(), 0, nil)
	if err != nil {
		return 0, err
	}

	// (ticks / ticksPerRotation) / 4 sec
	rps := (endPos - startPos) / ticksPerRotation / 4.0
	rpm := rps * 60.0
	return rpm, nil
}

func baseVelocityEstimate(b base.Base) (time.Duration, error) {
	var startTime, endTime time.Time
	for {
		move, err := b.IsMoving(context.Background())
		if err != nil {
			return 0, err
		}
		if move {
			startTime = time.Now()
			break
		}
	}
	for {
		move, err := b.IsMoving(context.Background())
		if err != nil {
			return 0, err
		}
		if !move {
			endTime = time.Now()
			break
		}
	}

	return endTime.Sub(startTime), nil
}

func motorVelocityEstimate(m motor.Motor) (time.Duration, error) {
	var startTime, endTime time.Time
	for {
		move, err := m.IsMoving(context.Background())
		if err != nil {
			return 0, err
		}
		if move {
			startTime = time.Now()
			break
		}
	}
	for {
		move, err := m.IsMoving(context.Background())
		if err != nil {
			return 0, err
		}
		if !move {
			endTime = time.Now()
			break
		}
	}

	return endTime.Sub(startTime), nil
}

func distBetweenAngles(endDeg, startDeg float64, originalDist int) float64 {
	totalDeg := endDeg - startDeg
	if totalDeg > 360 {
		totalDeg -= 360
	}
	if totalDeg < -360 {
		totalDeg += 360
	}
	turns := (originalDist / 360.0) % 360
	return totalDeg + float64(turns*360)
}
