"""On-demand debug log window for the Battery Digital Twin.

Message history is owned by the main app (a small capped buffer, always kept even while
no window is open); this window is just a live view onto it, opened from a header icon
button instead of taking a permanent slot in the sidebar -- day-to-day operation doesn't
need it, it's mainly useful when something needs debugging.
"""

import customtkinter as ctk

from theme import dual


class LogWindow(ctk.CTkToplevel):
    def __init__(self, master, lines, on_close=None):
        super().__init__(master)
        self.title("Debug Log")
        self.geometry("760x520")
        self.configure(fg_color=dual("app_bg"))
        self._onCloseCallback = on_close

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.txt = ctk.CTkTextbox(self, font=ctk.CTkFont(family="Consolas", size=11),
                                  fg_color=dual("card_bg_alt"), text_color=dual("text"),
                                  corner_radius=12, activate_scrollbars=True)
        self.txt.grid(row=0, column=0, sticky="nsew", padx=12, pady=(12, 6))
        self.set_lines(lines)

        ctk.CTkButton(self, text="Close", width=100, command=self._onClose).grid(
            row=1, column=0, sticky="e", padx=12, pady=(0, 12))

        self.protocol("WM_DELETE_WINDOW", self._onClose)

    def append(self, msg):
        self.txt.configure(state="normal")
        self.txt.insert("end", msg + "\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def set_lines(self, lines):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.insert("end", "\n".join(lines) + ("\n" if lines else ""))
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def _onClose(self):
        if self._onCloseCallback:
            self._onCloseCallback()
        self.destroy()
