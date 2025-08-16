import lgpio


h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, 12)
lgpio.gpio_write(h, 12, 1)
