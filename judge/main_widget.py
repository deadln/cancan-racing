from PyQt5.QtCore import QTimer, QTime, Qt
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import QWidget, QTableWidget, QGridLayout, QTableWidgetItem, QMessageBox

COLUMNS_WIDTH = 120


class MainWidget(QWidget):

    def __init__(self, formation):
        super().__init__()

        layout = QGridLayout()

        self.table = QTableWidget()
        self.table.setFixedWidth(COLUMNS_WIDTH * 4 + 2)
        self.table.horizontalHeader().hide()
        self.table.verticalHeader().hide()

        self.table.setColumnCount(4)
        self.table.setRowCount(6)
        for i in range(self.table.colorCount()):
            self.table.setColumnWidth(i, COLUMNS_WIDTH)

        self.timer = QTimer()
        self.time = QTime(0, 0)
        self.timer.timeout.connect(self.timer_event)
        self.timer.start(1000)

        timer_item = QTableWidgetItem("00:00")
        timer_item.setData(Qt.FontRole, QFont("", 20))
        timer_item.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.table.setSpan(0, 0, 1, 4)
        self.table.setItem(0, 0, timer_item)

        self.table.setSpan(1, 0, 1, 4)
        self.table.setItem(1, 0, QTableWidgetItem("Штрафы"))

        self.table.setItem(2, 0, QTableWidgetItem("Столкновения"))

        if formation:
            self.table.setItem(2, 2, QTableWidgetItem("Вылеты"))
        else:
            self.tableWidget.setItem(2, 2, QTableWidgetItem())
            self.tableWidget.setItem(2, 3, QTableWidgetItem())
            self.tableWidget.item(2, 2).setBackground(QColor(250, 250, 250))
            self.tableWidget.item(2, 3).setBackground(QColor(250, 250, 250))

        self.table.setSpan(3, 0, 1, 4)
        self.table.setItem(3, 0, QTableWidgetItem("Зоны"))

        if formation:
            self.table.setSpan(4, 0, 2, 1)
            self.table.setSpan(4, 1, 2, 1)
            self.table.setSpan(4, 2, 2, 1)
            self.table.setSpan(4, 3, 2, 1)
            self.table.setItem(4, 0, QTableWidgetItem("N"))
            self.table.setItem(4, 1, QTableWidgetItem("СКО\nтекущее"))
            self.table.setItem(4, 2, QTableWidgetItem("СКО\nсреднее"))
            self.table.setItem(4, 3, QTableWidgetItem("Время\nперестроения"))

        layout.addWidget(self.table)

        self.setLayout(layout)
        self.add_row()

    def timer_event(self):
        self.time = self.time.addSecs(1)
        timer_item = QTableWidgetItem(self.time.toString("mm:ss"))
        timer_item.setData(Qt.FontRole, QFont("", 20))
        timer_item.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.table.setItem(0, 0, timer_item)

    def add_row(self):
        rc = self.table.rowCount()
        self.table.insertRow(rc)
        self.table.setItem(rc, 0, QTableWidgetItem(str(rc - 5)))

    def update_collisions(self, data):
        self.table.setItem(2, 1, QTableWidgetItem(str(data)))

    def update_side_time(self, data):
        self.table.setItem(2, 3, QTableWidgetItem(str(round(float(data), 2))))

    def update_msd_current(self, data):
        self.table.setItem(self.table.rowCount() - 1, 1, QTableWidgetItem(str(round(float(data), 2))))

    def update_msd_overall(self, data):
        self.table.setItem(self.table.rowCount() - 1, 2, QTableWidgetItem(str(round(float(data), 2))))

    def update_reformation(self, data):
        self.table.setItem(self.table.rowCount() - 1, 3, QTableWidgetItem(str(round(float(data), 2))))

    def collisions_cb(self, data):
        self.update_collisions(data.data)

    def side_time_cb(self, data):
        self.update_side_time(data.data)

    def msd_current_cb(self, data):
        self.update_msd_current(data.data)

    def msd_overall_cb(self, data):
        self.update_msd_overall(data.data)
        self.add_row()

    def reformation_cb(self, data):
        self.update_reformation(data.data)

    # noinspection PyMethodMayBeStatic
    def final_cb(self, data):
        string = "FINAL SCORE: {} \n" \
                 "___________________________ \n" \
                 "NOMINATIONS\n" \
                 "SYNCHRONICITY: {} \n" \
                 "EFFICIENCY: {} \n" \
                 "SPEED: {}".format(*tuple(data.data.split('::')))
        print(string)
        QMessageBox.information(None, "RESULT", string, QMessageBox.Ok)
