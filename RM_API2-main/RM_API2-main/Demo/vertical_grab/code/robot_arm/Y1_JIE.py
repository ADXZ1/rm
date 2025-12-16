import time

from Class.Class_Delay import RelayController


if __name__ == '__main__':
    controller = RelayController(port='/dev/ttyUSB2', baudrate=38400, timeout=1)
    try:
        controller.turn_off_relay_Y1()
        time.sleep(1)
    finally:
        controller.close()