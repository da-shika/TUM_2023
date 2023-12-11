import os
from pathlib import Path

def main(dir):
    dir_path = Path(dir)

    if not dir_path.exists():
        print("Directory is not exist")
        return
    
    for file_path in dir_path.glob("**/*"):
        if file_path.is_file() and file_path.suffix.lower() == ".gif":
            try:
                file_path.unlink()
                print(f"{file_path} is Delete")
            except Exception as e:
                print(f"{file_path} is error: {e}")


if __name__ == "__main__":
    main("/home/genki/backup/python/tomm/final_experiment")