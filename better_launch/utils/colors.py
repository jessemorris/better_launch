import colorsys


class HighContrastColorGenerator:
    """A generator class that produces high-contrast RGB colors readable on black."""
    
    def __init__(self):
        self.golden_ratio_conjugate = 0.61803398875  # Ensures well-spaced hues
        self.hue = 0  # Start hue

    def __iter__(self):
        """Allows the class to be used as an iterable."""
        return self

    def __next__(self):
        """Generates the next high-contrast color."""
        self.hue = (self.hue + self.golden_ratio_conjugate) % 1  # Spread hues
        r, g, b = colorsys.hsv_to_rgb(self.hue, 1, 1)  # Full saturation and brightness
        return (int(r * 255), int(g * 255), int(b * 255))

    def __call__(self):
        """Allows calling the instance directly to get the next color."""
        return next(self)


get_contrast_color = HighContrastColorGenerator()
