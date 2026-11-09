"""Single serial owner with one outstanding query and a capacity-one mailbox."""

from threading import Event, Thread
from time import monotonic_ns

from gca.acquisition.buffers import Latest
from gca.domain.models import OperationResult
from gca.domain.position import EncoderAcquisitionSnapshot


class EncoderAcquisition:
    def __init__(self, reader, config, *, clock=monotonic_ns):
        self.reader, self.config, self.clock = reader, config, clock
        self._latest = Latest(EncoderAcquisitionSnapshot())
        self._stop, self._ready = Event(), Event()
        self._thread = None
        self._on_fault = None

    def snapshot(self):
        return self._latest.get()

    def start(self, on_fault=None):
        if self._thread is not None and self._thread.is_alive():
            return OperationResult(self.snapshot().healthy, self.snapshot().error_code or "OK")
        self._on_fault = on_fault
        self._stop.clear()
        self._ready.clear()
        self._latest.put(EncoderAcquisitionSnapshot())
        self._thread = Thread(target=self._run, name="gca-position-encoder-io", daemon=True)
        self._thread.start()
        if not self._ready.wait(1.0 + self.config.quiet_s + self.config.transaction_timeout_s):
            self.close()
            return OperationResult(False, "ENCODER_INITIALIZATION_TIMEOUT")
        status = self.snapshot()
        return OperationResult(status.healthy, status.error_code or "OK")

    def _run(self):
        try:
            self.reader.open()
            due = self.clock()
            period_ns = round(self.config.sample_period_s * 1e9)
            while not self._stop.is_set():
                if self._stop.wait(max(0.0, (due - self.clock()) / 1e9)):
                    break
                sample = self.reader.read(
                    self.clock() + round(self.config.transaction_timeout_s * 1e9)
                )
                if not sample.valid or sample.error_code:
                    raise RuntimeError(sample.error_code or "ENCODER_SAMPLE_INVALID")
                self._latest.put(EncoderAcquisitionSnapshot(sample, True, ""))
                self._ready.set()
                due += period_ns
                now = self.clock()
                if due < now:
                    due += ((now - due) // period_ns + 1) * period_ns
        except Exception as exc:
            code = getattr(exc, "code", "ENCODER_IO_ERROR")
            self._latest.put(EncoderAcquisitionSnapshot(self.snapshot().sample, False, code))
            self._ready.set()
            if self._on_fault is not None:
                self._on_fault(code)
        finally:
            self.reader.close()
            current = self.snapshot()
            if self._stop.is_set():
                self._latest.put(
                    EncoderAcquisitionSnapshot(current.sample, False, "ENCODER_CLOSED")
                )
            self._ready.set()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0 + self.config.transaction_timeout_s + self.config.quiet_s)
            if self._thread.is_alive():
                raise RuntimeError("encoder I/O owner did not close within its deadline")
