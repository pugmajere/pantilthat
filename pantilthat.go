package pantilthat

import "fmt"
import "github.com/go-daq/smbus"
import "log"
import "math"

/*
Caveats:
- LEDs are not currently supported.
*/

type PanTiltHatParams struct {
	Servo1Min, Servo1Max, Servo2Min, Servo2Max uint16
	IdleTimeout                                int
	Address                                    uint8
}

type PanTiltHat struct {
	params PanTiltHatParams

	// "zero" values are good defaults for these.
	servo1Timeout, servo2Timeout int
	enableServo1, enableServo2   bool

	// i2c is special
	i2cRetries   int
	i2cRetryTime float64
	i2cAddress   uint8
	i2cBus       *smbus.Conn
}

const (
	reg_config  = 0x00
	reg_servo1  = 0x01
	reg_servo2  = 0x03
	reg_ws2812  = 0x05 // unused.
	reg_update  = 0x4E
	update_wait = 0.03
)

func MakePanTiltHat(params *PanTiltHatParams) (*PanTiltHat, error) {
	hat := new(PanTiltHat)

	if params.Servo1Min == 0 {
		params.Servo1Min = 575
	}
	if params.Servo1Max == 0 {
		params.Servo1Max = 2325
	}
	if params.Servo2Min == 0 {
		params.Servo2Min = 575
	}
	if params.Servo2Max == 0 {
		params.Servo2Max = 2325
	}
	if params.IdleTimeout == 0 {
		params.IdleTimeout = 2
	}
	if params.Address == 0 {
		params.Address = 0x15
	}

	hat.params = *params

	hat.i2cRetries = 10
	hat.i2cRetryTime = 0.01
	hat.i2cAddress = params.Address
	i2cBus, err := smbus.Open(1, hat.i2cAddress)
	if err != nil {
		return nil, err
	}
	hat.i2cBus = i2cBus

	return hat, nil
}

func (hat *PanTiltHat) Close() {
	hat.enableServo1 = false
	hat.enableServo2 = false
	hat.setConfig()
}

func (hat *PanTiltHat) setConfig() error {
	var config uint8

	var enableServo1Bit, enableServo2Bit uint8
	if hat.enableServo1 {
		enableServo1Bit = 1
	}
	if hat.enableServo2 {
		enableServo2Bit = 1
	}
	config |= enableServo1Bit
	config |= enableServo2Bit << 1
	// Rest of the bits are used for lights, leaving at 0.
	err := hat.i2cBus.WriteReg(hat.i2cAddress, reg_config, config)

	return err
}

func (hat *PanTiltHat) ServoEnable(index uint8, state bool) error {
	if index < 1 || index > 2 {
		return fmt.Errorf("Servo index out of range: %d", index)
	}

	if index == 1 {
		hat.enableServo1 = state
	} else if index == 2 {
		hat.enableServo2 = state
	}

	err := hat.setConfig()
	if err != nil {
		return err
	}

	return nil
}

func (hat *PanTiltHat) ServoPulseMin(index uint8, value uint16) error {
	if index < 1 || index > 2 {
		return fmt.Errorf("Servo index out of range: %d", index)
	}

	if index == 1 {
		hat.params.Servo1Min = value
	} else if index == 2 {
		hat.params.Servo2Min = value
	}

	return nil
}

func (hat *PanTiltHat) ServoPulseMax(index uint8, value uint16) error {
	if index < 1 || index > 2 {
		return fmt.Errorf("Servo index out of range: %d", index)
	}

	if index == 1 {
		hat.params.Servo1Max = value
	} else if index == 2 {
		hat.params.Servo2Max = value
	}

	return nil
}

func servoUsToDegrees(us, usMin, usMax uint16) (int8, error) {
	if us < usMin || us > usMax {
		return 0, fmt.Errorf("pulse time outside expected range: %d vs [%d, %d]",
			us, usMin, usMax)
	}
	servoRange := usMax - usMin
	angle := (float64(us-usMin) / float64(servoRange)) * 180.0

	return int8(math.Trunc(angle+0.5)) - 90, nil
}

func servoDegreesToUs(degree int16, usMin, usMax uint16) (uint16, error) {
	if degree < -90 || degree > 90 {
		return 0, fmt.Errorf("Degree outside range: %i", degree)
	}

	degree += 90
	usRange := usMax - usMin
	us := (float64(usRange) / 180.0) * float64(degree)
	log.Printf("degree %d yielded us %f\n", degree, us)
	return usMin + uint16(us), nil
}

func (hat *PanTiltHat) GetServoOne() (int8, error) {
	value, err := hat.i2cBus.ReadWord(hat.i2cAddress, reg_servo1)
	if err != nil {
		return 0, err
	}

	// Convert pulse time in microseconds into degrees:

	return servoUsToDegrees(value, hat.params.Servo1Min, hat.params.Servo1Max)
}

func (hat *PanTiltHat) GetServoTwo() (int8, error) {
	value, err := hat.i2cBus.ReadWord(hat.i2cAddress, reg_servo2)
	if err != nil {
		return 0, err
	}

	// Convert pulse time in microseconds into degrees:
	return servoUsToDegrees(value, hat.params.Servo2Min, hat.params.Servo2Max)
}

func (hat *PanTiltHat) SetServoOne(angle int16) error {
	if !hat.enableServo1 {
		err := hat.ServoEnable(1, true)
		if err != nil {
			return err
		}
	}

	us, err := servoDegreesToUs(angle, hat.params.Servo1Min, hat.params.Servo1Max)
	if err != nil {
		return err
	}

	log.Printf("Writing %d to servo 1\n", us)
	err = hat.i2cBus.WriteWord(hat.i2cAddress, reg_servo1, us)
	// Consider idle handling here.
	return err
}

func (hat *PanTiltHat) SetServoTwo(angle int16) error {
	if !hat.enableServo2 {
		err := hat.ServoEnable(2, true)
		if err != nil {
			return err
		}
	}

	us, err := servoDegreesToUs(angle, hat.params.Servo2Min, hat.params.Servo2Max)
	if err != nil {
		return err
	}

	log.Printf("Writing %d to servo 2\n", us)
	err = hat.i2cBus.WriteWord(hat.i2cAddress, reg_servo2, us)
	// Consider idle handling here.
	return err
}

func (hat *PanTiltHat) Pan(angle int16) error {
	return hat.SetServoOne(angle)
}

func (hat *PanTiltHat) Tilt(angle int16) error {
	return hat.SetServoTwo(angle)
}
