package robot

import (
	"errors"
	"math"
)

// Robot contains the configuration and state of the delta robot
type Robot struct {
	BaseRadius          float64 `json:"BaseRadius"`
	BicepLength         float64 `json:"BicepLength"`
	ForearmLength       float64 `json:"ForearmLength"`
	EndEffectorRadius   float64 `json:"EndEffectorRadius"`
	BaseToFloorDistance float64 `json:"BaseToFloorDistance"`
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
	x0 := 0.0
	y0 := 0.0
	z0 := 0.0

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

	w1 := y1*y1 + z1*z1
	w2 := x2*x2 + y2*y2 + z2*z2
	w3 := x3*x3 + y3*y3 + z3*z3

	// x = (a1*z + b1)/dnm
	a1 := (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)
	b1 := -((w2-w1)*(y3-y1) - (w3-w1)*(y2-y1)) / 2.0

	// y = (a2*z + b2)/dnm
	a2 := -(z2-z1)*x3 + (z3-z1)*x2
	b2 := ((w2-w1)*x3 - (w3-w1)*x2) / 2.0

	// a*z^2 + b*z + c = 0
	a := a1*a1 + a2*a2 + dnm*dnm
	b := 2.0 * (a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
	c := (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1-r.ForearmLength*r.ForearmLength)

	// discriminant
	d := b*b - 4.0*a*c
	if d < 0.0 {
		return 0.0, 0.0, 0.0, errors.New("Invalid position")
	}

	z0 = -0.5 * (b + math.Sqrt(d)) / a
	x0 = (a1*z0 + b1) / dnm
	y0 = (a2*z0 + b2) / dnm

	return x0, y0, z0, nil
}

// Inverse kinematics (x,y,z) -> (theta1, theta2, theta3)
func (r *Robot) Inverse(x, y, z float64) (float64, float64, float64, error) {
	theta1, err := r.calcAngleYZ(x, y, z)
	if err != nil {
		return 0.0, 0.0, 0.0, err
	}

	theta2, err := r.calcAngleYZ((x*cos120)+(y*sin120), (y*cos120)-(x*sin120), z)
	if err != nil {
		return 0.0, 0.0, 0.0, err
	}

	theta3, err := r.calcAngleYZ((x*cos120)-(y*sin120), (y*cos120)+(x*sin120), z)
	if err != nil {
		return 0.0, 0.0, 0.0, err
	}

	return theta1, theta2, theta3, err

}

// helper function, calculates angle theta1 (for YZ-pane)
func (r *Robot) calcAngleYZ(x0, y0, z0 float64) (float64, error) {
	y1 := -0.5 * tan30 * r.BaseRadius
	y0 -= 0.5 * tan30 * r.EndEffectorRadius

	// z = a + b*y
	a := (x0*x0 + y0*y0 + z0*z0 + r.BicepLength*r.BicepLength - r.ForearmLength*r.ForearmLength - y1*y1) / (2.0 * z0)
	b := (y1 - y0) / z0

	// discriminant
	d := -(a+b*y1)*(a+b*y1) + r.BicepLength*(b*b*r.BicepLength+r.BicepLength)
	if d < 0 {
		return 0.0, errors.New("Invalid position")
	}

	yj := (y1 - a*b - math.Sqrt(d)) / (b*b + 1)
	zj := a + b*yj

	theta := math.Atan(-zj/(y1-yj))*180.0/math.Pi + 0.0
	if yj > y1 {
		theta = math.Atan(-zj/(y1-yj))*180.0/math.Pi + 180.0
	}

	return theta, nil
}
