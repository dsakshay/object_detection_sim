#!/usr/bin/env python3

import logging
from datetime import datetime

def myprodlog(level ,msg=''):
    # if level == logging.CRITICAL:
    #     logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.CRITICAL)
    # elif level == logging.DEBUG:
    #     logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
    # elif level == logging.ERROR:
    #     logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.ERROR)
    # elif level == logging.INFO:
    #     logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    logger = logging.getLogger("log_test_node")
    logger2 = logging.getLogger("log_test_node2")
    file = "/home/akshay/es_ws/" + str(datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")) + ".log"
    logging.basicConfig(filename=file, format='%(name)s:%(asctime)s:%(levelname)s:%(message)s')
    logger.setLevel(level)

    # file = "/home/akshay/es_ws/examples.log"
    # logging.basicConfig(filename=file, format='%(name)s:%(levelname)s:%(message)s')
    logger2.setLevel(level)
    # logging.debug(msg)
    # logging.info(msg)
    # logging.error(msg)
    match level:
        case logging.DEBUG:

    
            logger.debug(msg)
            logger2.debug(msg)

        case logging.INFO:
            logger.info(msg)
            logger2.info(msg)

        case logging.ERROR:
            logger.error(msg)
            logger2.error(msg)
    # logger.info(msg)
    # logger.error(msg)


if __name__ == "__main__":
    myprodlog(logging.ERROR, "Here it is!")
    myprodlog(logging.INFO, "Info here -")

