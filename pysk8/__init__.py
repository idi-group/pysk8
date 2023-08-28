import logging
import sys

from . import core

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

def setCoreLogLevel(lvl: int) -> None:
    """
    Shortcut method to set pysk8.core loglevel
    
    Args: 
        lvl (int): a standard level enum from the logging module

    Returns:
        nothing
    """
    core.logger.setLevel(lvl)    
