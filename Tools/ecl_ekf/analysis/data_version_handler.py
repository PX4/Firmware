#! /usr/bin/env python3
"""
function collection for handling different versions of log files
"""
from pyulog import ULog

from analysis.detectors import PreconditionError

def get_output_tracking_error_message(ulog: ULog) -> str:
    """
    return the name of the message containing the output_tracking_error
    :param ulog:
    :return: str
    """
    for elem in ulog.data_list:
        if elem.name == "ekf2_innovations":
            return "ekf2_innovations"
        if elem.name == "estimator_innovations":
            return "estimator_status"

    raise PreconditionError("Could not detect the message containing the output tracking error")

def get_innovation_message(ulog: ULog, topic: str = 'innovation') -> str:
    """
    return the name of the innovation message (old: ekf2_innovations; new: estimator_innovations)
    :param ulog:
    :return: str
    """
    if topic == 'innovation':
        for elem in  ulog.data_list:
            if elem.name == "ekf2_innovations":
                return "ekf2_innovations"
            if elem.name == "estimator_innovations":
                return "estimator_innovations"
    if topic == 'innovation_variance':
        for elem in  ulog.data_list:
            if elem.name == "ekf2_innovations":
                return "ekf2_innovations"
            if elem.name == "estimator_innovations":
                return "estimator_innovations"
    if topic == 'innovation_test_ratio':
        for elem in  ulog.data_list:
            if elem.name == "ekf2_innovations":
                return "ekf2_innovations"
            if elem.name == "estimator_innovations":
                return "estimator_innovations"

    raise PreconditionError("Could not detect the message")
