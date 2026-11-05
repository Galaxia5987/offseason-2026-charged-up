from synapse.core.pipeline import Pipeline, PipelineResult
from synapse.core.settings_api import NumberConstraint, PipelineSettings, settingField

from pipeline import pipeline


class ColorDetectionSettings(PipelineSettings):
    minimum_percentage = settingField(
        NumberConstraint(minValue=0.1, maxValue=100, step=0.1),
        default=25.0,
        category="Detection",
        description="Minimum percentage of frame pixels that must match a color",
    )

    tone = settingField(
        NumberConstraint(minValue=0, maxValue=100, step=1),
        default=50.0,
        category="Camera",
        description="Color tone: green at 0, neutral at 50, magenta at 100",
    )

    temperature = settingField(
        NumberConstraint(minValue=0, maxValue=100, step=1),
        default=50.0,
        category="Camera",
        description="Color temperature: cool at 0, neutral at 50, warm at 100",
    )


class ColorPipeLine(Pipeline[ColorDetectionSettings, PipelineResult]):
    def __init__(self, settings: ColorDetectionSettings):
        super().__init__(settings)

    def processFrame(self, img, timestamp: float):
        minimum_percentage = self.getSetting(self.settings.minimum_percentage)
        tone = self.getSetting(self.settings.tone)
        temperature = self.getSetting(self.settings.temperature)

        img, green, yellow, red = pipeline(
            img,
            minimum_percentage,
            tone,
            temperature,
        )

        self.setDataValue("has_green_cube", green)
        self.setDataValue("has_yellow_cube", yellow)
        self.setDataValue("has_red_cube", red)

        return img
