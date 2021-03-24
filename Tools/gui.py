#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


"""
gui.py:
PX4 pyside Qt GUI confgure to Kconfig and compile and flash boards

@author: Peter van der Perk <peter.vanderperk@nxp.com>
"""

import sys
from pprint import pprint as pp

from PySide2.QtWidgets import QApplication, QWidget, QGridLayout, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QSizePolicy, QTreeView, QGroupBox, QTextEdit, QComboBox, QItemDelegate, QMenu, QMessageBox
from PySide2.QtGui import QStandardItem, QStandardItemModel, QPixmap, QFont
from PySide2.QtCore import QObject, Signal, QFile, Qt, QProcessEnvironment, QProcess, QSortFilterProxyModel, QAbstractProxyModel
from PySide2.QtUiTools import QUiLoader

from kconfiglib import Kconfig, standard_kconfig, Symbol, Choice, MENU, COMMENT, MenuNode, \
                       BOOL, TRISTATE, STRING, INT, HEX, \
                       AND, OR, expr_value

class KconfigItemDelegate(QItemDelegate):
    """
    Custom QItemDelegate that creates a QComboBox editor
    when a Kconfig choice option is selected
    """
    def __init__(self, parent):
        QItemDelegate.__init__(self, parent)

    def createEditor(self, parent, option, index):
        nodeItem = index.data(Qt.UserRole)
        if isinstance(nodeItem, Symbol):
            return QItemDelegate.createEditor(self, parent, option, index)
        elif isinstance(nodeItem, Choice):
            combo = QComboBox(parent)
            selectIndex = 0
            for choiceItem in nodeItem.syms:
                for node in choiceItem.nodes:
                    if node.item.visibility != 0 and node.prompt:
                        combo.addItem(node.prompt[0])
                        if nodeItem.selection == node:
                            selectIndex = combo.count()-1
            combo.setCurrentIndex(selectIndex)
            return combo
        return None

    def setEditorData(self, editor, index):
        nodeItem = index.data(Qt.UserRole)
        if isinstance(nodeItem, Symbol):
            return QItemDelegate.setEditorData(self, editor, index)
        elif isinstance(nodeItem, Choice):
            text = index.data()
            index = editor.findText(text)
            editor.setCurrentIndex(index)

    def setModelData(self, editor, model, index):
        nodeItem = index.data(Qt.UserRole)
        if isinstance(nodeItem, Symbol):
            return QItemDelegate.setModelData(self, editor, model, index)
        elif isinstance(nodeItem, Choice):
            model.setData(index, editor.itemText(editor.currentIndex()))

    def updateEditorGeometry(self, editor, option, index):
        nodeItem = index.data(Qt.UserRole)
        if isinstance(nodeItem, Symbol):
            return QItemDelegate.updateEditorGeometry(self, editor, option, index)
        elif isinstance(nodeItem, Choice):
            editor.setGeometry(option.rect)

class KconfigFilterProxyModel(QSortFilterProxyModel):
    """
    Custom QSortFilterProxyModel that hides invisible kconfig symbols
    using Kconfiblib
    """

    def __init__(self, parent=None):
        super(KconfigFilterProxyModel, self).__init__(parent)

    def filterAcceptsRow(self, source_row, source_parent):
        nodeItem = self.sourceModel().index(source_row, 0, source_parent).data(Qt.UserRole)
        if isinstance(nodeItem, Symbol) or isinstance(nodeItem, Choice):
            if(nodeItem.visibility == 0):
                return False
        return True

class Kconfig(QObject):
    """
    Kconfig class to provide a Kconfig QStandardItemModel for QTreeView
    """
    items = []

    def __init__(self):
        super(Kconfig, self).__init__()
        env = QProcessEnvironment.systemEnvironment()
        self.defconfig_path = env.value('GUI_DEFCONFIG')
        self.kconf = standard_kconfig(env.value('GUI_KCONFIG'))
        self.kconf.load_config(self.defconfig_path)

        self.model = QStandardItemModel()
        self.model.setRowCount(0)
        self.getItems(self.kconf.top_node.list, self.model.invisibleRootItem())
        self.updateItems()

        self.kconfigProxyModel = KconfigFilterProxyModel()
        self.kconfigProxyModel.setSourceModel(self.model)

        self.model.itemChanged.connect(self.configChanged)

    def saveDefConfig(self):
        print(self.kconf.write_min_config(self.defconfig_path))

    def configChanged(self, field: QStandardItem):
        nodeItem = field.data(Qt.UserRole);
        if isinstance(nodeItem, Symbol):
            if(field.isCheckable()):
                if(field.checkState() == Qt.Checked):
                    nodeItem.set_value(2) # Why 2 though??
                elif(field.checkState() == Qt.PartiallyChecked):
                    nodeItem.set_value(1)
                else:
                    nodeItem.set_value(0)
            elif(field.isEditable()):
                if(nodeItem.set_value(field.text()) == False): # failed set back to orig_type
                    field.setText(nodeItem.str_value)
        if isinstance(nodeItem, Choice):
            for choiceItem in nodeItem.syms:
                for node in choiceItem.nodes:
                    if node.prompt:
                        if field.text() == node.prompt[0]:
                            choiceItem.set_value(2) # Set this symbol to Y, kconfiglib does the rest
                            break

        self.updateItems()
        self.kconfigProxyModel.invalidateFilter()

    def getModel(self):
        return self.kconfigProxyModel

    def updateItems(self):
        for qItem in self.items:
            nodeItem = qItem.data(Qt.UserRole);
            if isinstance(nodeItem, Symbol):
                if(qItem.isCheckable()):
                    if(nodeItem.user_value is not None):
                        if(nodeItem.user_value == 2):
                            qItem.setCheckState(Qt.Checked)
                        elif(nodeItem.user_value == 1):
                            qItem.setCheckState(Qt.PartiallyChecked)
                        else:
                            qItem.setCheckState(Qt.Unchecked)
                    else: # Set default value
                        if(nodeItem.str_value == 'y'):
                            qItem.setCheckState(Qt.Checked)
                        elif(nodeItem.str_value == 'm'):
                            qItem.setCheckState(Qt.PartiallyChecked)
                        else:
                            qItem.setCheckState(Qt.Unchecked)
                else:
                    index = self.model.indexFromItem(qItem).siblingAtColumn(1)
                    qValueItem = self.model.itemFromIndex(index)
                    qValueItem.setText(nodeItem.str_value)
            elif isinstance(nodeItem, Choice):
                index = self.model.indexFromItem(qItem).siblingAtColumn(1)
                qValueItem = self.model.itemFromIndex(index)
                for choiceItem in nodeItem.syms:
                    for node in choiceItem.nodes:
                        if node.prompt:
                            if nodeItem.selection == node.item:
                                qValueItem.setText(node.prompt[0])

    def getItems(self, node, parent):
        while node:
            qnodeItem = None
            if node.item == MENU:
                qnodeItem = QStandardItem(node.prompt[0])
                qnodeItem.setEditable(False)
                parent.appendRow([qnodeItem, QStandardItem()])
                qnodeItem.setData(node.item, Qt.UserRole) # Menu are always visible
            elif isinstance(node.item, Choice) and node.list is not None:
                qchoiceItem = QStandardItem(node.prompt[0])
                qchoiceItem.setEditable(False)
                qchoiceItemOption = QStandardItem()
                qchoiceItemOption.setData(node.item, Qt.UserRole)

                parent.appendRow([qchoiceItem, qchoiceItemOption])
                qchoiceItem.setData(node.item.name, Qt.ToolTipRole)
                qchoiceItem.setData(node.item, Qt.UserRole)
                self.items.append(qchoiceItem)
            elif hasattr(node.item, 'orig_type') and node.item.orig_type in (BOOL,TRISTATE):
                if node.prompt is not None:
                    qnodeItem = QStandardItem(node.prompt[0])
                    qnodeItem.setEditable(False)
                    qnodeItem.setCheckable(True)

                    if(node.item.orig_type == TRISTATE):
                        qnodeItem.setCheckable(True)
                        qnodeItem.setTristate(True)
                        qnodeItem.setUserTristate(True)

                    parent.appendRow([qnodeItem, QStandardItem()])
                    qnodeItem.setData(node.item.name, Qt.ToolTipRole)
                    qnodeItem.setData(node.item, Qt.UserRole)
                    self.items.append(qnodeItem)
            elif isinstance(node.item, Symbol):
                if node.prompt is not None:
                    qnodeItem = QStandardItem(node.prompt[0])
                    qnodeItem.setEditable(False)

                    # Int string hex
                    qnodeValue = QStandardItem()
                    qnodeValue.setData(node.item, Qt.UserRole)

                    parent.appendRow([qnodeItem, qnodeValue])
                    qnodeItem.setData(node.item.name, Qt.ToolTipRole)
                    qnodeItem.setData(node.item, Qt.UserRole)
                    self.items.append(qnodeItem)

            if node.list:
                if(qnodeItem is not None):
                    self.getItems(node.list, qnodeItem)

            node = node.next

    # Function to add right click menu to treeview item
    def openMenu(self, position):
        modelIndex = self.kconfigProxyModel.mapToSource(self.sender().indexAt(position))
        item = self.model.itemFromIndex(modelIndex)
        nodeItem = item.data(Qt.UserRole)
        if isinstance(nodeItem, Symbol) or isinstance(nodeItem, Choice):
            right_click_menu = QMenu()
            act_default = right_click_menu.addAction(self.tr("Set to default"))
            act_default.triggered.connect(lambda: self.setDefault(nodeItem))
            act_help = right_click_menu.addAction(self.tr("Help"))
            act_help.triggered.connect(lambda: self.helpDialog(nodeItem))
            right_click_menu.exec_(self.sender().viewport().mapToGlobal(position))

    def setDefault(self, nodeItem):
        nodeItem.unset_value()
        self.updateItems()
        self.kconfigProxyModel.invalidateFilter()

    def helpDialog(self, nodeItem):
        s = ''
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        for node in nodeItem.nodes:
            if node.help is not None:
                    s += node.help
        msg.setText(s)
        msg.exec_()


class PX4DevGUI(QWidget):
    taskRunning = False

    def __init__(self):
        super().__init__()
        env = QProcessEnvironment.systemEnvironment()
        self.config = env.value('CONFIG')

        layoutGrid = QGridLayout()
        self.setLayout(layoutGrid)

        # Board info name, platform target
        boardInfoLayout = QHBoxLayout()
        layoutGrid.addLayout(boardInfoLayout, 0, 0)

        boardInfoLabel = QLabel("%s %s\n"
                                "Label: %s\n"
                                "Platform: %s\n"
                                "%s\n"
                                "Architecture: %s\n"
                                "Romfsroot: %s\n"
                                % (env.value('VENDOR'),env.value('MODEL'),env.value('LABEL'),
                                   env.value('PLATFORM'),env.value('TOOLCHAIN'),
                                   env.value('ARCHITECTURE'),env.value('ROMFSROOT')));
        boardImageLabel = QLabel('Insert board image here');
        boardPixmap = QPixmap("Tools/" + self.config + ".png") #FIXME proper way to store board images
        boardImageLabel.setPixmap(boardPixmap)
        boardImageLabel.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter);

        boardInfoLayout.addWidget(boardInfoLabel)
        boardInfoLayout.addWidget(boardImageLabel)

        # Kconfiglib based configuration options
        self.dataGroupBox = QGroupBox("configuration")
        self.dataView = QTreeView(self)
        self.dataView.setHeaderHidden(True)

        dataLayout = QHBoxLayout()
        dataLayout.addWidget(self.dataView)
        self.dataGroupBox.setLayout(dataLayout)

        self.kconfig = Kconfig()
        self.dataView.setModel(self.kconfig.getModel())
        self.dataView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.dataView.customContextMenuRequested.connect(self.kconfig.openMenu)
        self.dataView.setItemDelegateForColumn(1, KconfigItemDelegate(self.dataView))
        self.dataView.expanded.connect(lambda: self.dataView.resizeColumnToContents(0))
        self.dataView.resizeColumnToContents(0);

        kconfigLayout = QVBoxLayout()
        kconfigLayout.addWidget(self.dataGroupBox)
        layoutGrid.addLayout(kconfigLayout, 1, 0)

        # CMake output
        self.terminalGroupBox = QGroupBox("Terminal output")
        self.terminalOutput = QTextEdit()
        f = QFont("unexistent")
        f.setStyleHint(QFont.Monospace);
        self.terminalOutput.setFont(f);
        self.terminalOutput.setReadOnly(True)

        terminalLayout = QHBoxLayout()
        terminalLayout.addWidget(self.terminalOutput)
        self.terminalGroupBox.setLayout(terminalLayout)

        terminalLayout = QVBoxLayout()
        terminalLayout.addWidget(self.terminalGroupBox)
        layoutGrid.addLayout(terminalLayout, 2, 0)

        # GUI actions configure, compile & flash
        actionLayout = QHBoxLayout()
        layoutGrid.addLayout(actionLayout, 3, 0)

        actionConfigure = QPushButton('Configure');
        actionConfigure.clicked.connect(lambda: self.configureClick())
        actionCompile = QPushButton('Compile');
        actionCompile.clicked.connect(lambda: self.compileClick())
        actionFlash = QPushButton('Flash');
        actionFlash.clicked.connect(lambda: self.flashClick())
        px4Logo = QLabel('PX4 Logo');
        px4logopixmap = QPixmap('Tools/PX4-Logo-Black.png')
        px4Logo.setPixmap(px4logopixmap)
        px4Logo.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter);

        actionLayout.addWidget(actionConfigure)
        actionLayout.addWidget(actionCompile)
        actionLayout.addWidget(actionFlash)
        actionLayout.addWidget(px4Logo)

    def __del__(self):
        if(self.taskRunning == True):
            self.myProcess.terminate()
            self.myProcess.waitForFinished()

    def taskFinished(self):
        print("Task done") #TODO Make a loading circle and process return status??
        self.taskRunning = False;

    def runMake(self, suffix = None):
        if(self.taskRunning == False):
            self.taskRunning = True;

            program = "make"
            arguments = [self.config]

            if suffix is not None:
                arguments.append(suffix)

            self.myProcess = QProcess(self)
            self.myProcess.start(program, arguments)
            self.myProcess.setProcessEnvironment(QProcessEnvironment.systemEnvironment())
            self.myProcess.readyReadStandardOutput.connect(self.write_terminal_output)
            self.myProcess.finished.connect(self.taskFinished)

    def configureClick(self):
        self.kconfig.saveDefConfig()
        self.runMake("clean")

    def compileClick(self):
        self.runMake()

    def flashClick(self):
        self.runMake("upload")

    def write_terminal_output(self):
        self.terminalOutput.append(self.myProcess.readAllStandardOutput().data().decode())

def main():
    app = QApplication(sys.argv)
    demo = PX4DevGUI()
    demo.show()
    sys.exit(app.exec_())

main()
