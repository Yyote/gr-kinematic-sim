def try_get(fn, default):
    try:
        val = fn().value
        if val is None:
            return default
        else:
            return val
    except:
        return default
