package robot

import (
	"errors"
	"math"
)

// Robot contains the configuration and state of the delta robot
type Robot struct {
	BaseRadius          float64 `json:"f"`
	BicepLength         float64 `json:"rf"`
	ForearmLength       float64 `json:"re"`
	EndEffectorRadius   float64 `json:"e"`
	BaseToFloorDistance float64 `json:"b"`

	Motor1AngleMin float64
	Motor1AngleMax float64
	Motor2AngleMin float64
	Motor2AngleMax float64
	Motor3AngleMin float64
	Motor3AngleMax float64
}

// RectangularCuboidEnvelope defines the envelope of the delta robot
type RectangularCuboidEnvelope struct {
	Xmin float64
	Xmax float64
	Ymin float64
	Ymax float64
	Zmin float64
	Zmax float64
}

// Trigonometric constants
const (
	sqrt3  = 1.73205080757
	sin120 = sqrt3 / 2.0
	cos120 = -0.5
	tan60  = sqrt3
	sin30  = 0.5
	tan30  = 1.0 / sqrt3
	dtr    = math.Pi / 180.0
)

// Forward kinematics (theta1, theta2, theta3) -> (x, y, z, error)
// returns
func (r *Robot) Forward(theta1, theta2, theta3 float64) (float64, float64, float64, error) {
	x := 0.0
	y := 0.0
	z := 0.0

	t := (r.BaseRadius - r.EndEffectorRadius) * tan30 / 2.0

	theta1 *= dtr
	theta2 *= dtr
	theta3 *= dtr

	y1 := -(t + r.BicepLength*math.Cos(theta1))
	z1 := -r.BicepLength * math.Sin(theta1)

	y2 := (t + r.BicepLength*math.Cos(theta2)) * sin30
	x2 := y2 * tan60
	z2 := -r.BicepLength * math.Sin(theta2)

	y3 := (t + r.BicepLength*math.Cos(theta3)) * sin30
	x3 := -y3 * tan60
	z3 := -r.BicepLength * math.Sin(theta3)

	dnm := (y2-y1)*x3 - (y3-y1)*x2

	w1 := (y1 * y1) + (z1 * z1)
	w2 := (x2 * x2) + (y2 * y2) + (z2 * z2)
	w3 := (x3 * x3) + (y3 * y3) + (z3 * z3)

	// x = (a1*z + b1)/dnm
	a1 := ((z2 - z1) * (y3 - y1)) - ((z3 - z1) * (y2 - y1))
	b1 := (-((w2 - w1) * (y3 - y1)) - ((w3-w1)*(y2-y1))/2.0)

	// y = (a2*z + b2)/dnm;
	a2 := (-(z2 - z1) * x3) + ((z3 - z1) * x2)
	b2 := (((w2 - w1) * x3) - ((w3-w1)*x2)/2.0)

	// a*z^2 + b*z + c = 0
	a := (a1 * a1) + (a2 * a2) + (dnm * dnm)
	b := (2.0*(a1*b1) + (a2 * (b2 - y1*dnm)) - (z1 * dnm * dnm))
	c := ((b2 - (y1 * dnm)) * (b2 - y1*dnm)) + (b1 * b1) + dnm*dnm*((z1*z1)-(r.ForearmLength*r.ForearmLength))

	// discriminant
	d := (b * b) - (4.0 * a * c)
	if d < 0.0 {
		return 0.0, 0.0, 0.0, errors.New("Non-existing povar.")
	}

	z = -0.5 * (b + math.Sqrt(d)) / a
	x = (a1*z + b1) / dnm
	y = (a2*z + b2) / dnm

	return x, y, z, nil
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
func (r *Robot) calcAngleYZ(x0, y0, z0 float64) (float64, error) {
	y1 := -0.5 * tan30 * r.BaseRadius
	y0 -= 0.5 * tan30 * r.EndEffectorRadius

	// z = a + b*y
	a := ((x0 * x0) + (y0 * y0) + (z0 * z0) + (r.BicepLength * r.BicepLength) - (r.ForearmLength * r.ForearmLength) - (y1 * y1)) / (2.0 * z0)
	b := (y1 - y0) / z0

	// discriminant
	d := (-(a + b*y1) * (a + b*y1)) + (r.BicepLength * (b*b*r.BicepLength + r.BicepLength))
	if d < 0 {
		return 0.0, errors.New("Non-existing povar.")
	}

	yj := (y1 - (a * b) - math.Sqrt(d)) / ((b * b) + 1.0)
	zj := a + (b * yj)

	theta := math.Atan((-zj/(y1-yj))*180.0/math.Pi) + 0.0

	if yj > y1 {
		theta = math.Atan((-zj/(y1-yj))*180.0/math.Pi) + 180.0
	}

	return theta, nil
}
