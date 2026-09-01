"""Čistá rozhodovací logika pro automatické odpojení OUT (Load) relé s hysterezí."""


def decide_relay_action(voltage_sum, is_on, off_v, on_v):
    """
    Vrátí True/False pokud se má relé přepnout do daného stavu, jinak None (žádná akce).
    Hystereze: vypnout při poklesu na/pod off_v, zapnout zpět při nárůstu na/nad on_v.
    """
    if voltage_sum != voltage_sum:  # NaN — neplatné čtení, neriskovat akci naslepo
        return None
    if is_on and voltage_sum <= off_v:
        return False
    if not is_on and voltage_sum >= on_v:
        return True
    return None
