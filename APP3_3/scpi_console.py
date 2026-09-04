"""Manual SCPI console for the Battery Digital Twin — diagnostics directly on the serial port."""

import customtkinter as ctk

from theme import dual


class ScpiConsoleWindow(ctk.CTkToplevel):
    VOLTAGE_RANGES = ["15V", "1V5", "0V15"]
    RESISTANCE_RANGES = ["200k", "3k3"]
    SPEEDS = ["Fast", "Slow"]
    STATES = ["1 (ON)", "0 (OFF)"]
    COMMANDS = ["MEASure:VOLTage?", "MEASure:RESistance?", "SET:OUTput"]

    def __init__(self, master, sendCommandFn):
        super().__init__(master)
        self.title("SCPI Console")
        self.geometry("620x580")
        self.configure(fg_color=dual("app_bg"))
        self.sendCommandFn = sendCommandFn  # callable(cmd: str) -> str | None

        # -- Quick command builder --
        builder = ctk.CTkFrame(self, corner_radius=14, fg_color=dual("card_bg"),
                                border_width=1, border_color=dual("border"))
        builder.pack(fill="x", padx=16, pady=(16, 8))
        ctk.CTkLabel(builder, text="Quick command builder", font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=12, pady=(10, 6))

        row1 = ctk.CTkFrame(builder, fg_color="transparent")
        row1.pack(fill="x", padx=12, pady=4)
        ctk.CTkLabel(row1, text="Command", width=70, anchor="w",
                     text_color=dual("text_secondary")).pack(side="left")
        self.cmdVar = ctk.StringVar(value=self.COMMANDS[0])
        self.cmdMenu = ctk.CTkOptionMenu(row1, values=self.COMMANDS, variable=self.cmdVar,
                                          command=lambda _v: self._onCommandChange())
        self.cmdMenu.pack(side="left", fill="x", expand=True, padx=(8, 0))

        row2 = ctk.CTkFrame(builder, fg_color="transparent")
        row2.pack(fill="x", padx=12, pady=4)
        ctk.CTkLabel(row2, text="Channel", width=70, anchor="w",
                     text_color=dual("text_secondary")).pack(side="left")
        self.channelEntry = ctk.CTkEntry(row2, width=80)
        self.channelEntry.insert(0, "1")
        self.channelEntry.pack(side="left", padx=(8, 0))
        self.channelEntry.bind("<KeyRelease>", lambda e: self._rebuildPreview())

        row3 = ctk.CTkFrame(builder, fg_color="transparent")
        row3.pack(fill="x", padx=12, pady=4)
        self.optionLabel = ctk.CTkLabel(row3, text="Range", width=70, anchor="w",
                                         text_color=dual("text_secondary"))
        self.optionLabel.pack(side="left")
        self.optionVar = ctk.StringVar(value=self.VOLTAGE_RANGES[0])
        self.optionMenu = ctk.CTkOptionMenu(row3, values=self.VOLTAGE_RANGES, variable=self.optionVar,
                                             command=lambda _v: self._rebuildPreview())
        self.optionMenu.pack(side="left", fill="x", expand=True, padx=(8, 0))

        self.row4 = ctk.CTkFrame(builder, fg_color="transparent")
        self.row4.pack(fill="x", padx=12, pady=(4, 4))
        self.speedLabel = ctk.CTkLabel(self.row4, text="Speed", width=70, anchor="w",
                                        text_color=dual("text_secondary"))
        self.speedLabel.pack(side="left")
        self.speedVar = ctk.StringVar(value=self.SPEEDS[0])
        self.speedMenu = ctk.CTkOptionMenu(self.row4, values=self.SPEEDS, variable=self.speedVar,
                                            command=lambda _v: self._rebuildPreview())
        self.speedMenu.pack(side="left", fill="x", expand=True, padx=(8, 0))

        ctk.CTkLabel(builder,
                     text="Your selection is mirrored into the line below — feel free to edit it "
                          "by hand before sending.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=560, justify="left").pack(anchor="w", padx=12, pady=(4, 10))

        # -- Output and command line --
        self.output = ctk.CTkTextbox(self, font=ctk.CTkFont(family="Consolas", size=11),
                                      fg_color=dual("card_bg"), text_color=dual("text"),
                                      corner_radius=14)
        self.output.pack(fill="both", expand=True, padx=16, pady=8)
        self.output.configure(state="disabled")

        entryRow = ctk.CTkFrame(self, fg_color="transparent")
        entryRow.pack(fill="x", padx=16, pady=(0, 16))
        self.entry = ctk.CTkEntry(entryRow, placeholder_text="e.g. MEASure:VOLTage5? 15V,Fast")
        self.entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.entry.bind("<Return>", lambda e: self.send())
        ctk.CTkButton(entryRow, text="Send", width=90, command=self.send).pack(side="left")

        self._onCommandChange()
        self.log("Console ready. Pick a command above, or type one in by hand.")

    def _onCommandChange(self):
        cmd = self.cmdVar.get()
        if cmd == "MEASure:VOLTage?":
            self.optionLabel.configure(text="Range")
            self.optionMenu.configure(values=self.VOLTAGE_RANGES)
            self.optionVar.set(self.VOLTAGE_RANGES[0])
            self.row4.pack(fill="x", padx=12, pady=(4, 4))
        elif cmd == "MEASure:RESistance?":
            self.optionLabel.configure(text="Range")
            self.optionMenu.configure(values=self.RESISTANCE_RANGES)
            self.optionVar.set(self.RESISTANCE_RANGES[0])
            self.row4.pack(fill="x", padx=12, pady=(4, 4))
        else:  # SET:OUTput
            self.optionLabel.configure(text="State")
            self.optionMenu.configure(values=self.STATES)
            self.optionVar.set(self.STATES[0])
            self.row4.pack_forget()
        self._rebuildPreview()

    def _rebuildPreview(self):
        cmd = self.cmdVar.get()
        ch = self.channelEntry.get().strip() or "1"

        if cmd == "MEASure:VOLTage?":
            text = f"MEASure:VOLTage{ch}? {self.optionVar.get()},{self.speedVar.get()}"
        elif cmd == "MEASure:RESistance?":
            text = f"MEASure:RESistance{ch}? {self.optionVar.get()},{self.speedVar.get()}"
        else:
            state = "1" if self.optionVar.get().startswith("1") else "0"
            text = f"SET:OUTput{ch} {state}"

        self.entry.delete(0, "end")
        self.entry.insert(0, text)

    def log(self, text):
        self.output.configure(state="normal")
        self.output.insert("end", text + "\n")
        self.output.see("end")
        self.output.configure(state="disabled")

    def send(self):
        cmd = self.entry.get().strip()
        if not cmd:
            return
        self.log(f"»  {cmd}")
        resp = self.sendCommandFn(cmd)
        if resp is None:
            self.log("   [error] port not connected")
        else:
            self.log(f"«  {resp}")
