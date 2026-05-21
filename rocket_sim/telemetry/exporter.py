from __future__ import annotations

from pathlib import Path

import pandas as pd


def export_dataframe_to_csv(df: pd.DataFrame, output_path: str | Path) -> Path:
    """Export telemetry to CSV and create the output directory if needed."""

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)
    return output_path
