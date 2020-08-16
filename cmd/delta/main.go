package main

import (
	"os"

	"github.com/jessevdk/go-flags"
)

type Opts struct {
	BaseRadius          float64 `short:"b" long:"base-radius" description:"Base radius" default:"75.0" value-name:"mm"`
	BicepLength         float64 `short:"r" long:"bicep-length" description:"Bicep length" default:"100.0" value-name:"mm"`
	ForearmLength       float64 `short:"f" long:"forearm-length" description:"Forearm length" default:"300.0" value-name:"mm"`
	EndEffectorRadius   float64 `short:"e" long:"end-effector-radius" description:"End effector radius" default:"24.0" value-name:"mm"`
	BaseToFloorDistance float64 `short:"d" long:"base-to-floor-distance" description:"End effector radius" default:"400.0" value-name:"mm"`
}

var opts Opts
var parser = flags.NewParser(&opts, flags.Default)

func main() {
	if _, err := parser.Parse(); err != nil {
		if flagsErr, ok := err.(*flags.Error); ok && flagsErr.Type == flags.ErrHelp {
			os.Exit(0)
		} else {
			os.Exit(1)
		}
	}
}
