package robot

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestForward(t *testing.T) {
	r := &Robot{
		BaseRadius:          75.0,
		BicepLength:         100.0,
		ForearmLength:       300.0,
		EndEffectorRadius:   24.0,
		BaseToFloorDistance: 400.0,
	}

	x, y, z, err := r.Forward(5.0, 10.0, 15.0)
	assert.Nil(t, err)
	assert.InDelta(t, x, 12.188, 0.01)
	assert.InDelta(t, y, -21.871, 0.01)
	assert.InDelta(t, z, -285.025, 0.01)

	x, y, z, err = r.Forward(0.0, 0.0, 0.0)
	assert.Nil(t, err)
	assert.InDelta(t, x, 0.0, 0.01)
	assert.InDelta(t, y, 0.0, 0.01)
	assert.InDelta(t, z, -277.198, 0.01)
}
