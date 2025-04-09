from autonav_uwb import UWBTag

if __name__ == "__main__":

    tag = UWBTag(port='/dev/ttyTHS1', baudrate=115200, debug=True)

    def new_distance(*args, **kwargs):
        
        print(f"new distance: {kwargs}")

    tag.on(tag.Event.DISTANCE, new_distance)

    try:
        tag.start()
    except KeyboardInterrupt:
        tag.stop()


