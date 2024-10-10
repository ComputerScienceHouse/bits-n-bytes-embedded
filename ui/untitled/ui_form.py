# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main.ui'
##
## Created by: Qt User Interface Compiler version 6.7.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QListView, QSizePolicy, QStackedWidget,
    QWidget)

class Ui_main(object):
    def setupUi(self, main):
        if not main.objectName():
            main.setObjectName(u"main")
        main.resize(1024, 600)
        self.Main = QStackedWidget(main)
        self.Main.setObjectName(u"Main")
        self.Main.setGeometry(QRect(0, 0, 1024, 600))
        self.Welcome = QWidget()
        self.Welcome.setObjectName(u"Welcome")
        self.Main.addWidget(self.Welcome)
        self.Cart = QWidget()
        self.Cart.setObjectName(u"Cart")
        self.listView = QListView(self.Cart)
        self.listView.setObjectName(u"listView")
        self.listView.setGeometry(QRect(10, 60, 660, 540))
        self.Main.addWidget(self.Cart)

        self.retranslateUi(main)

        self.Main.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(main)
    # setupUi

    def retranslateUi(self, main):
        main.setWindowTitle(QCoreApplication.translate("main", u"main", None))
    # retranslateUi

