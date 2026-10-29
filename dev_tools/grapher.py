from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication, QScrollArea


class ValueGrapher:
    series: dict[str, dict[str, deque[tuple[float, float]]]]
    plots: dict[str, pg.PlotItem]
    curves: dict[tuple[str, str], pg.PlotDataItem]
    max_points: int
    cols: int
    row_height: int
    app: QApplication
    window: QScrollArea
    layout: pg.GraphicsLayoutWidget

    def __init__(self, max_points: int, title: str = "", cols: int = 1, row_height: int = 200) -> None:
        self.series = {}
        self.plots = {}
        self.curves = {}
        self.max_points = max_points
        self.cols = cols
        self.row_height = row_height

        self.app = pg.mkQApp(title)
        self.layout = pg.GraphicsLayoutWidget()
        self.window = QScrollArea()
        self.window.setWidgetResizable(True)
        self.window.setWidget(self.layout)
        self.window.setWindowTitle(title)
        self.window.resize(1400, 800)
        self.window.show()

    def add_point(self, graph: str, point: tuple[float, float], line: str | None = None) -> None:
        line = line or graph
        lines = self.series.setdefault(graph, {})

        if line not in lines:
            lines[line] = deque(maxlen=self.max_points)

        lines[line].append(point)

    def update(self) -> None:
        for graph, lines in self.series.items():
            plot = self.plots.get(graph) or self._add_plot(graph)

            for line, points in lines.items():
                if not points:
                    continue

                curve = self.curves.get((graph, line)) or self._add_curve(plot, graph, line)
                data = np.array(points, dtype=float)
                curve.setData(data[:, 0], data[:, 1])

            plot.legend.setVisible(len(lines) > 1)

        self.app.processEvents()

    def is_open(self) -> bool:
        return self.window.isVisible()

    def _add_plot(self, graph: str) -> pg.PlotItem:
        row, col = divmod(len(self.plots), self.cols)
        plot = self.layout.addPlot(row=row, col=col, title=graph)
        plot.showGrid(x=True, y=True)
        _ = plot.addLegend(offset=(5, 5))

        self.layout.setMinimumHeight((row + 1) * self.row_height)
        self.plots[graph] = plot
        return plot

    def _add_curve(self, plot: pg.PlotItem, graph: str, line: str) -> pg.PlotDataItem:
        colour = pg.intColor(list(self.series[graph]).index(line), hues=9)
        curve = plot.plot(name=line, pen=colour)

        self.curves[graph, line] = curve
        return curve
