"""Hardware adapters with explicit lifecycle and lazily loaded dependencies.

Only the runtime I/O owner may call an adapter. Constructors never connect,
configure pins or enable torque. Unit tests inject transport/SDK factories.
"""
