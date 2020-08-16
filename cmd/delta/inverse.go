package main

import (
	"fmt"
	"log"

	"github.com/borud/delta/pkg/robot"
)

type InverseCommand struct {
	X float64 `short:"x" description:"X position" default:"0.0" value-name:"X Position in mm"`
	Y float64 `short:"y" description:"Y position" default:"0.0" value-name:"Y Position in mm"`
	Z float64 `short:"z" description:"Z position" default:"250.0" value-name:"Z Position in mm"`
}

func init() {
	parser.AddCommand(
		"inverse",
		"Inverse Kinematics",
		"Inverse Kinematics calculations",
		&InverseCommand{})
}

func (f *InverseCommand) Execute(args []string) error {
	r := &robot.Robot{
		BaseRadius:          opts.BaseRadius,
		BicepLength:         opts.BicepLength,
		ForearmLength:       opts.ForearmLength,
		EndEffectorRadius:   opts.EndEffectorRadius,
		BaseToFloorDistance: opts.BaseToFloorDistance,
	}

	t1, t2, t3, err := r.Inverse(f.X, f.Y, f.Z)
	if err != nil {
		log.Fatalf("Error computing inverse kinematics: %s", err)
	}

	fmt.Printf("theta1 = %.3f\ntheta2 = %.3f\ntheta3 = %.3f\n", t1, t2, t3)

	return nil
}
