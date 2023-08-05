import sys


def filter_doxygen_skip(filename):
    try:
        with open(filename, "r") as file:
            for line in file:
                # Check if the line contains "DOXYGEN_SKIP" and replace it with an empty line
                if "#include <" in line:
                    line = "\n"
                print(line, end="")
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except PermissionError:
        print(f"Error: Permission denied to access file '{filename}'.")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py <filename>")
    else:
        filename = sys.argv[1]
        filter_doxygen_skip(filename)
