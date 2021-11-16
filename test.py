import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Point import Point

#generate layout
app = QtGui.QApplication([])  # 初始化app
win = pg.GraphicsLayoutWidget(show=True)  # 设置widget，并属性show为True，即显示
win.setWindowTitle('pyqtgraph example: crosshair')  # 设置窗口名称
label = pg.LabelItem(justify='right')  # 设置label用于跟随鼠标显示横纵坐标有效值
win.addItem(label)  # 定义了label之后，要addItem之后才会真正加进去
p1 = win.addPlot(row=1, col=0)  # p1为p2的区域放大，并显示鼠标所在位置值
p2 = win.addPlot(row=2, col=0)  # p2为全部数据画出的折线致密的原图

# 定义区域
region = pg.LinearRegionItem()
region.setZValue(10)  # 不过这一句话好像没什么作用
# Add the LinearRegionItem to the ViewBox, but tell the ViewBox to exclude this
# item when doing auto-range calculations.
p2.addItem(region, ignoreBounds=True)

#pg.dbg()
p1.setAutoVisible(y=True)

data1 = 10000 + 15000 * pg.gaussianFilter(np.random.random(size=10000), 10) + 3000 * np.random.random(size=10000)
data2 = 15000 + 15000 * pg.gaussianFilter(np.random.random(size=10000), 10) + 3000 * np.random.random(size=10000)

# 画图
p1.plot(data1, pen="r")
p1.plot(data2, pen="g")

p2.plot(data1, pen="w")

# 设置图2中放大区域被移动后的触发函数
def update():
    region.setZValue(10)
    minX, maxX = region.getRegion()  # 调整p1的横轴显示区域坐标范围
    p1.setXRange(minX, maxX, padding=0)

region.sigRegionChanged.connect(update)

def updateRegion(window, viewRange):
    rgn = viewRange[0]
    region.setRegion(rgn)

p1.sigRangeChanged.connect(updateRegion)

vLine = pg.InfiniteLine(angle=90, movable=False)  # angle控制线相对x轴正向的相对夹角
hLine = pg.InfiniteLine(angle=0, movable=False)
p1.addItem(vLine, ignoreBounds=True)
p1.addItem(hLine, ignoreBounds=True)


vb = p1.vb

# 跟随鼠标移动，提取鼠标的横轴值，并自定义纵轴的值显示
def mouseMoved(evt):
    pos = evt[0]  ## using signal proxy turns original arguments into a tuple
    if p1.sceneBoundingRect().contains(pos):
        mousePoint = vb.mapSceneToView(pos)
        # 建议不用int，精度高时用float，这样可以显示横坐标的小数
        index = int(mousePoint.x())
        if index > 0 and index < len(data1):
            label.setText("<span style='font-size: 12pt'>x=%0.1f,   <span style='color: red'>y1=%0.1f</span>,   <span style='color: green'>y2=%0.1f</span>" % (mousePoint.x(), data1[index], data2[index]))
        vLine.setPos(mousePoint.x())
        hLine.setPos(mousePoint.y())

# 设置鼠标移动的触发，限制速率，移动则触发mouseMoved函数
proxy = pg.SignalProxy(p1.scene().sigMouseMoved, rateLimit=60, slot=mouseMoved)
#p1.scene().sigMouseMoved.connect(mouseMoved)


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()