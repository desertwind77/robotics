#!/usr/bin/env python3
'''
The module for utility functions
'''

import logging
import signal
import sys
import threading


class SignalException(Exception):
   '''An exception used to terminate our multithreaded application'''

class HardwareException(Exception):
   '''An excetipn caused by hardware error'''


class ThreadBase(threading.Thread):
   '''Base class for our threads'''
   def __init__(self) -> None:
      super(ThreadBase, self).__init__()
      self.name = 'ThreadBase'
      self.isExiting = threading.Event()

   def shutdown(self) -> None:
      logging.info('Shutting down {}'.format(self.name))
      self.isExiting.set()

   def run(self) -> None:
      assert False, "This function expected to be implemented in a child class"


def raise_term_exception(signum, stack) -> Exception:
   '''Raise SignalException so that our multithreaded programs can
   catch this exception and exit gracefully

	Args:
      signum (int): the signal number that was caught

      stack (frame): the stack frame
   '''
   logging.info('Caught signal {}'.format(signum))
   raise SignalException


def install_signal_handler() -> None:
   '''Catch SIGINT and SIGTERM and terminate the program'''
   signal.signal(signal.SIGINT, raise_term_exception)
   signal.signal(signal.SIGTERM, raise_term_exception)


def setup_logging(filename: str = None, verbose: bool = False) -> None:
    '''Set up logging facility

    Args:
        filename (str): the name of the log file

        verbose (bool): print debug information, as well.

    Returns:
        None
    '''
    file_params = {
        'filename': filename,
        'filemode': 'a',
    }
    stream_params = {
        'stream': sys.stdout
    }
    format_params = {
        'format': '%(asctime)s - %(levelname)s - %(message)s',
        'level': logging.DEBUG if verbose else logging.INFO,
    }
    if filename is not None:
        log_params = {**file_params, **format_params}
    else:
        log_params = {**stream_params, **format_params}

    logging.basicConfig(**log_params)
