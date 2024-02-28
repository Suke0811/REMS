try:
    import pynput
except:
    raise ImportError("You need to install rems keyboard packages with 'pip install rems[keyboard]'")


from .key.KeyboardInput import KeyboardInput

