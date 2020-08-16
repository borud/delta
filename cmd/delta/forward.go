package main

import (
	"fmt"
	"log"

	"github.com/borud/delta/pkg/robot"
)

type ForwardCommand struct {
	Theta1 float64 `long:"t1" description:"Theta 1 angle" default:"0.0" value-name:"Angle in degrees"`
	Theta2 float64 `long:"t2" description:"Theta 2 angle" default:"0.0" value-name:"Angle in degrees"`
	Theta3 float64 `long:"t3" description:"Theta 3 angle" default:"0.0" value-name:"Angle in degrees"`
}

func init() {
	parser.AddCommand(
		"forward",
		"Forward Kinematics",
		"Forward Kinematics calculations",
		&ForwardCommand{})
}

func (f *ForwardCommand) Execute(args []string) error {
	r := &robot.Robot{
		BaseRadius:          opts.BaseRadius,
		BicepLength:         opts.BicepLength,
		ForearmLength:       opts.ForearmLength,
		EndEffectorRadius:   opts.EndEffectorRadius,
		BaseToFloorDistance: opts.BaseToFloorDistance,
	}

	x, y, z, err := r.Forward(f.Theta1, f.Theta2, f.Theta3)
	if err != nil {
		log.Fatalf("Error computing forward kinematics: %s", err)
	}

	fmt.Printf("x = %.3f\ny = %.3f\nz = %.3f\n", x, y, z)

	return nil
}
