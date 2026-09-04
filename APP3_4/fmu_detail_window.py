import bisect

import customtkinter as ctk
from matplotlib import colormaps
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from theme import GRAY, RED, dual, tokens_for_mode


class FmuDetailWindow(ctk.CTkToplevel):
    """Standalone window with per-cell detail of the thermal model's 24 cells -- its own
    Y axis (unlike the main chart, where temperature shares an axis with 6 real sensors
    and a small simulated range tends to disappear)."""

    def __init__(self, master, nCells):
        super().__init__(master)
        self.title("Thermal Model — 24-Cell Detail")
        self.geometry("1000x720")
        self.nCells = nCells
        self._onCloseCallback = None

        self.fig = Figure(dpi=100)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[1, 2.6], hspace=0.28)
        self.axP = self.fig.add_subplot(gs[0])
        self.axT = self.fig.add_subplot(gs[1], sharex=self.axP)
        self.axP.set_ylabel("P_loss [W]\n(1 cell)")
        self.axT.set_ylabel("Temperature [°C]")
        self.axT.set_xlabel("Time [s]")
        self.axP.tick_params(labelbottom=False)
        self.fig.subplots_adjust(left=0.08, right=0.86, top=0.96, bottom=0.08)

        (self.pLine,) = self.axP.plot([], [], "-", linewidth=1.2, color=RED, label="P_loss (ECM)")
        self.axP.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8,
                         borderaxespad=0.0, frameon=True)

        cmap = colormaps["turbo"]
        self.cellLines = []
        for k in range(nCells):
            color = cmap(k / max(1, nCells - 1))
            (line,) = self.axT.plot([], [], "-", linewidth=1.1, color=color, label=f"Cell {k + 1}")
            self.cellLines.append(line)
        self.axT.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7,
                         borderaxespad=0.0, frameon=True, ncol=1, labelspacing=0.35)

        chartFrame = ctk.CTkFrame(self, fg_color="transparent")
        chartFrame.pack(fill="both", expand=True, padx=10, pady=(10, 4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=chartFrame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        toolbarFrame = ctk.CTkFrame(self, fg_color="transparent")
        toolbarFrame.pack(fill="x", padx=10, pady=(0, 2))
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbarFrame)
        self.toolbar.update()

        hintFrame = ctk.CTkFrame(self, fg_color="transparent")
        hintFrame.pack(fill="x", padx=10, pady=(0, 10))
        ctk.CTkLabel(hintFrame,
                     text="The temperature axis scales to the simulation only (not to the real "
                          "sensors), so a small range is still visible. Use the magnifier in the "
                          "toolbar above to zoom into any region (both axes); the home button "
                          "restores the full range.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=940, justify="left").pack(anchor="w")

        # -- Crosshair with value readout, same approach as the main window --
        self._tData, self._pLossY = [], []
        self._fmuTData, self._fmuY = [], [[] for _ in range(nCells)]
        self._vlines, self._readouts = {}, {}
        for ax in (self.axP, self.axT):
            vline = ax.axvline(x=0, color=GRAY, linewidth=0.8, linestyle="--", visible=False)
            text = ax.text(0.012, 0.96, "", transform=ax.transAxes, va="top", ha="left",
                           fontsize=8, visible=False,
                           bbox=dict(boxstyle="round,pad=0.4", alpha=0.95))
            self._vlines[ax] = vline
            self._readouts[ax] = text
        self.canvas.mpl_connect("motion_notify_event", self._onChartHover)
        self.canvas.mpl_connect("figure_leave_event", self._hideCrosshair)

        self.applyTheme()
        self.protocol("WM_DELETE_WINDOW", self._onClose)

    # ------------------------------------------------------------------
    def _onChartHover(self, event):
        if not self._fmuTData or event.inaxes not in self._vlines or event.xdata is None:
            self._hideCrosshair()
            return

        def nearest(xs, x):
            i = bisect.bisect_left(xs, x)
            if i >= len(xs):
                return len(xs) - 1
            if i > 0 and abs(xs[i - 1] - x) < abs(xs[i] - x):
                return i - 1
            return i

        pIdx = nearest(self._tData, event.xdata) if self._tData else None
        tIdx = nearest(self._fmuTData, event.xdata)
        t = self._fmuTData[tIdx]

        if pIdx is not None and pIdx < len(self._pLossY):
            p = self._pLossY[pIdx]
            pText = f"t = {self._tData[pIdx]:.1f} s\nP_loss: {p:.3f} W" if p == p \
                else f"t = {self._tData[pIdx]:.1f} s\nP_loss: ---"
        else:
            pText = f"t = {t:.1f} s"
        self._readouts[self.axP].set_text(pText)

        cellLines = [f"t = {t:.1f} s"]
        for k, line in enumerate(self.cellLines):
            y = self._fmuY[k] if k < len(self._fmuY) else []
            v = y[tIdx] if tIdx < len(y) else float("nan")
            cellLines.append(f"{line.get_label()}: {v:.2f} °C" if v == v
                             else f"{line.get_label()}: ---")
        self._readouts[self.axT].set_text("\n".join(cellLines))

        for ax, vline in self._vlines.items():
            vline.set_xdata([t, t])
            vline.set_visible(True)
        for text in self._readouts.values():
            text.set_visible(True)
        self.canvas.draw_idle()

    def _hideCrosshair(self, event=None):
        changed = False
        for vline in self._vlines.values():
            if vline.get_visible():
                vline.set_visible(False)
                changed = True
        for text in self._readouts.values():
            if text.get_visible():
                text.set_visible(False)
                changed = True
        if changed:
            self.canvas.draw_idle()

    def applyTheme(self):
        t = tokens_for_mode(ctk.get_appearance_mode())
        self.fig.set_facecolor(t["card_bg"])
        for ax in (self.axP, self.axT):
            ax.set_facecolor(t["card_bg"])
            ax.tick_params(colors=t["text_secondary"])
            ax.xaxis.label.set_color(t["text"])
            ax.yaxis.label.set_color(t["text"])
            for spine in ax.spines.values():
                spine.set_visible(False)
            ax.grid(True, color=t["separator"], linewidth=0.6, alpha=0.8)
            legend = ax.get_legend()
            if legend is not None:
                frame = legend.get_frame()
                frame.set_facecolor(t["card_bg"])
                frame.set_edgecolor(t["border"])
                frame.set_alpha(1.0)
                for text in legend.get_texts():
                    text.set_color(t["text"])
            self._vlines[ax].set_color(t["text_secondary"])
            readout = self._readouts[ax]
            readout.set_color(t["text"])
            patch = readout.get_bbox_patch()
            if patch is not None:
                patch.set_facecolor(t["card_bg"])
                patch.set_edgecolor(t["border"])
        self.canvas.draw_idle()

    def update_data(self, tData, pLossY, fmuTData, fmuY):
        self._tData, self._pLossY = tData, pLossY
        self._fmuTData, self._fmuY = fmuTData, fmuY
        self.pLine.set_data(tData, pLossY)
        for k, line in enumerate(self.cellLines):
            line.set_data(fmuTData, fmuY[k] if k < len(fmuY) else [])
        for ax in (self.axP, self.axT):
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    def set_on_close(self, callback):
        self._onCloseCallback = callback

    def _onClose(self):
        if self._onCloseCallback:
            self._onCloseCallback()
        self.destroy()
