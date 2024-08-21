# rulerscale.py
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QFont
from PySide6.QtCore import Qt

class RulerScale(QWidget):
    def __init__(self, parent=None, min_value=-40, max_value=40, interval=10):
        super(RulerScale, self).__init__(parent)
        self.min_value = min_value
        self.max_value = max_value
        self.interval = interval
        self.minor_tick_height = 5
        self.major_tick_height = 10

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        total_range = self.max_value - self.min_value
        step_width = width / total_range

        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        painter.setFont(QFont('Arial', 8))

        for value in range(self.min_value, self.max_value + 1):
            x = (value - self.min_value) * step_width
            if value % self.interval == 0:
                painter.drawLine(x, height - self.major_tick_height, x, height)
                if value != 0:  # Skip drawing the 0 label
                    painter.drawText(x - 10, height - self.major_tick_height - 5, str(value))
            else:
                painter.drawLine(x, height - self.minor_tick_height, x, height)

        painter.end()
