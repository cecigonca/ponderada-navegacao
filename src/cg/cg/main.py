import rclpy
import threading

from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor
from .cli import main as main

def game():
    rclpy.init()
    game = Game(load_from_csv("maps/default.csv"))
    cli_thread = threading.Thread(target=main)
    cli_thread.start()
    thread = threading.Thread(target=rclpy.spin, args=(game,))
    thread.start()

    try:
        while rclpy.ok():
            game.run()
    except KeyboardInterrupt:   
        pass
    finally:
        game.destroy_node()
        rclpy.shutdown()
        cli_thread.join()
        thread.join()

def editor():
    editor = Editor(load_from_csv("maps/default.csv"))
    editor.run()