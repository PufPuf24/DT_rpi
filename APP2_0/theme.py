"""Sdílená Apple-like light/dark paleta pro Battery Digital Twin a jeho okna."""

LIGHT = {
    "app_bg": "#F2F2F7",
    "card_bg": "#FFFFFF",
    "card_bg_alt": "#F2F2F7",
    "border": "#E1E1E6",
    "text": "#1D1D1F",
    "text_secondary": "#6E6E73",
    "separator": "#D8D8DC",
}
DARK = {
    "app_bg": "#151516",
    "card_bg": "#1E1E20",
    "card_bg_alt": "#2A2A2C",
    "border": "#333335",
    "text": "#F5F5F7",
    "text_secondary": "#98989D",
    "separator": "#333335",
}

ACCENT = "#0A84FF"
GREEN = "#30D158"
RED = "#FF453A"
ORANGE = "#FF9F0A"
YELLOW = "#FFD60A"
GRAY = "#8E8E93"


def dual(key):
    """Vrátí (light, dark) dvojici pro CTk widget — sám si podle motivu vybere správnou."""
    return (LIGHT[key], DARK[key])


def tokens_for_mode(mode):
    return LIGHT if mode == "Light" else DARK
