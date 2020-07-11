import serial
import sys
import time


potential_serials = [
    '/dev/ttyACM0',
    '/dev/ttyACM1',
    '/dev/ttyUSB0',
    '/dev/ttyUSB1'
]


class OuchException(Exception):
    """
    Raised when the robot does not return 'OK' (0). This implies that the
    robot bumped into something and was unable to complete the command.
    """
    pass


class Robot:
    """
    Class for commanding the robot. Commands are encoded in the following
    format:
    - forward(x centimetres):- 'fx'
    - backward(x centimetres):- 'bx'
    - turn_left(x degrees):- 'lx'
    - turn_right(x degrees):- 'rx'
    - pen_up():- 'u'
    - pen_down():- 'd'

    The Robot class maintains a connection to the Arduino using a pyserial
    object. This handles reading/writing over the UART interface. When a
    command is sent to the Arduino, the method will block until a response
    is received - either 0 (OK) or 1 (error). A response of 1 generally
    implies that the robot hit an object and could not move any further, so
    the command failed. It is up to the user to handle the exception that
    is raised when this occurs.

    If at any point the serial connection is terminated, the program will
    display a message containing possible solutions and exit.
    """

    def __init__(self, serial_port='', baud_rate=115200):
        """
        Creates a new Robot object. Robots can be controlled with several
        different methods (like forward/backward etc.) similarly to Turtle.

        :param serial_port: The USB port for establishing the serial
                            connection. If unset, cycle through several
                            potential ports (e.g. /dev/ttyACM*, /dev/ttyUSB*
                            etc.) It is unlikely that this will need changing.
        :param baud_rate:   The baud rate for the serial connection. This MUST
                            match the baud rate used on the Arduino end of
                            the connection (which is set to 115200 by
                            default).
        """
        # Repeatedly attempt to connect - ensures that the user is given the
        # chance to plug the cable in after running their code
        connected = False
        i = 0
        actual_port = serial_port
        while not connected:
            try:
                if not serial_port:
                    actual_port = potential_serials[i]
                self._arduino_serial = serial.Serial(
                    actual_port,
                    baud_rate)
                connected = True
            except serial.serialutil.SerialException:
                self._handle_serial_error(should_exit=False)
                time.sleep(1)
                i += 1

        # Block until the Arduino sends a 'start' code. This code should NOT
        # be 0 or 1, as these are reserved for command responses. This
        # effectively acts as a 'synchronisation' phase, ensuring that
        # commands are not sent until the Arduino is ready.
        response = -1
        while response != 2:
            response = self._read()
            time.sleep(1)
            self._write('2')
            time.sleep(1)

        print('[Robot] Received response from Arduino. Beginning commands...')

    def _write(self, data):
        """
        Writes a string to the Arduino over the UART serial.

        :param data:    The string to send.
        """
        try:
            self._arduino_serial.write(data.encode('ascii'))
        except serial.serialutil.SerialException:
            self._handle_serial_error()

    def _read(self):
        """
        Reads a byte from the UART serial.

        :return:    The int value of the received byte.
        """
        response = -1

        try:
            response = self._arduino_serial.read(1)
        except serial.serialutil.SerialException:
            self._handle_serial_error()

        # Ignore non-integer or non-ascii responses, these could be debug logs
        # Commands will repeatedly attempt to read until they get a desired
        # response anyway
        try:
            response = int(response.decode('ascii'))
        except ValueError:
            response = -1
        except UnicodeDecodeError:
            response = -1

        return response

    def _handle_serial_error(self, should_exit=True):
        """
        Handles exceptions raised by the serial object.

        :param should_exit: Whether or not the program should terminate after
                            a serial exception.
        """
        print(
            '[Robot] Unable to connect to the robot. Please check that the '
            'Arduino is plugged in to the Raspberry Pi and that the serial '
            'port is correctly set.')

        if should_exit:
            sys.exit(1)

    def _handle_response(self):
        """
        Blocks until a response from the Arduino is received.

        :raises OuchException:  If the Arduino returns 1 (error).
        """
        response = -1
        while response != 0 and response != 1:
            response = self._read()

        if response == 0:
            print('[Robot] Command completed!')

        if response == 1:
            raise OuchException('Ouch!')

    def forward(self, amount):
        """
        Instructs the Arduino to move the robot forward.

        :param amount:  The number of centimetres to move by.

        :raises OuchException:  If the Arduino returns 1 (error).
        """
        print(
            '[Robot] Moving forward {amount} centimetres'.format(
                amount=amount))

        self._write('f' + str(amount) + '\n')
        self._handle_response()

    def backward(self, amount):
        """
        Instructs the Arduino to move the robot backward.

        :param amount:  The number of centimetres to move by.

        :raises OuchException:  If the Arduino returns 1 (error).
        """
        print(
            '[Robot] Moving backward {amount} centimetres'.format(
                amount=amount))

        self._write('b' + str(amount) + '\n')
        self._handle_response()

    def turn_left(self, amount):
        """
        Instructs the Arduino to turn the robot left.

        :param amount:  The number of degrees to move by.

        :raises OuchException:  If the Arduino returns 1 (error).
        """
        print('[Robot] Turning left {amount} degrees'.format(amount=amount))

        self._write('l' + str(amount) + '\n')
        self._handle_response()

    def turn_right(self, amount):
        """
        Instructs the Arduino to turn the robot right.

        :param amount:  The number of degrees to move by.

        :raises OuchException:  If the Arduino returns 1 (error).
        """
        print('[Robot] Turning right {amount} degrees'.format(amount=amount))

        self._write('r' + str(amount) + '\n')
        self._handle_response()

    def pen_up(self):
        """
        Instructs the Arduino to lift the pen up.
        """
        print('[Robot] Lifting pen up')

        self._write('u\n')
        self._handle_response()

    def pen_down(self):
        """
        Instructs the Arduino to put the pen down.
        """
        print('[Robot] Putting pen down')

        self._write('d\n')
        self._handle_response()


if __name__ == '__main__':
    robot = Robot()
    # Run UMBmark test
    while True:
        robot.pen_down()
        robot.forward(100)
        robot.turn_right(90)
        robot.forward(100)
        robot.turn_right(90)
        robot.forward(100)
        robot.turn_right(90)
        robot.forward(100)
        time.sleep(10)
        robot.turn_right(180)
        robot.forward(100)
        robot.turn_left(90)
        robot.forward(100)
        robot.turn_left(90)
        robot.forward(100)
        robot.turn_left(90)
        robot.forward(100)
        time.sleep(60)
