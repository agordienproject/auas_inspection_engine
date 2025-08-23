"""
Custom spin box widget that ignores mouse wheel events.
"""
from PyQt5.QtWidgets import QSpinBox

class NoWheelSpinBox(QSpinBox):
    """QSpinBox that ignores mouse wheel events to prevent accidental changes.

    This is useful for forms where focused spin boxes may be scrolled past
    and inadvertently altered by the user.
    """

    def wheelEvent(self, event): # The function is not snake case because it overrides a Qt method
        """Ignore mouse wheel events."""
        event.ignore()
