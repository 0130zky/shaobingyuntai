"""Compatibility imports; protocol and transport now have separate dependencies."""
from telemetry_protocol import (TAIL, FIELDS, DIAG_TAIL, DIAG_FIELDS, RECORD_FIELDS,
                                Decoder, valid_values, valid_diagnostics)
from serial_transport import SerialPort
