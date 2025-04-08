import sqlite3
import pandas as pd
from pathlib import Path


def main():
    fid = Path('~/cobra/m30.db').expanduser()
    db = sqlite3.connect(fid)
    table = pd.read_sql_query("SELECT * from sensorheaddump", db)
    print(table.columns)


if __name__ == "__main__":
    main()
