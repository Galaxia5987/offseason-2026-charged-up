```mermaid
stateDiagram-v2
    INTAKING --> IDLE :!intakeButton
    IDLE --> ALIGNMENT :scoringButton.trigger
    ALIGNMENT --> IDLE :scoringButton.trigger
    ALIGNMENT --> COLOR_CHECK :onComplete
    COLOR_CHECK --> SCORING_LOW :isGreen
    COLOR_CHECK --> SCORING_HIGH :!isGreen
    SCORING_LOW --> IDLE :onComplete
    SCORING_LOW --> IDLE :scoringButton.trigger
    SCORING_HIGH --> IDLE :onComplete
    SCORING_HIGH --> IDLE :scoringButton.trigger
```
