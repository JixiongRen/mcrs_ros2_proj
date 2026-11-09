"""Position-only component construction; construction never opens a device."""

from time import monotonic_ns

from gca.devices.common import EncoderConfig
from gca.devices.encoder import SerialEncoder
from gca.domain.position import PositionComponents

from .acquisition import EncoderAcquisition
from .feedback import EncoderFeedback
from .simulation import PositionSimulationDrive, SimulatedPositionEncoder
from .strategy import EncoderPositionStrategy


def create_position_components(config, drive=None, *, clock=monotonic_ns, slip_ratio=1.0):
    if config.control_mode != "position" or config.position is None:
        raise ValueError("position-mode configuration required")
    encoder = config.position.encoder
    if config.backend == "simulation":
        if not isinstance(drive, PositionSimulationDrive):
            drive = PositionSimulationDrive(config, clock=clock, slip_ratio=slip_ratio, drive=drive)
        reader = SimulatedPositionEncoder(drive, encoder, clock=clock)
    else:

        def serial_factory(**kwargs):
            # Keep the serial dependency lazy, including in manual installations.
            from serial import Serial

            return Serial(
                **kwargs,
                exclusive=True,
                bytesize=encoder.bytesize,
                parity=encoder.parity,
                stopbits=encoder.stopbits,
            )

        reader = SerialEncoder(
            EncoderConfig(
                port=encoder.port,
                baudrate=encoder.baudrate,
                address=encoder.address,
                register=encoder.register_start,
                bits=encoder.bits,
                signed=encoder.signed,
                quiet_s=encoder.quiet_s,
                byteorder=encoder.byte_order_in_register,
                modulus=encoder.count_modulus,
                transaction_timeout_s=encoder.transaction_timeout_s,
                verified=True,
            ),
            clock=clock,
            serial_factory=serial_factory,
        )
    return drive, PositionComponents(
        EncoderAcquisition(reader, encoder, clock=clock),
        EncoderFeedback(encoder),
        EncoderPositionStrategy(config),
    )
