from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

from ui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Painel de Controle do Robô")

    window = MainWindow()
    window.resize(1320, 820)
    window.show()
    QTimer.singleShot(0, window.showMaximized)
    QTimer.singleShot(500, window.ble.scan)

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
