import argparse

parser = argparse.ArgumentParser(description="Use Reinforced DMP to adapt to new goals")
parser.add_argument('-of', '--output-file', type=str, default='output.txt',
                    help="Output trajectory file")
arg = parser.parse_args()
