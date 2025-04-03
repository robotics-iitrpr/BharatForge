import sqlite3
import argparse

db_path = 'tasks.db'

def add_bot(namespace):
    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()
        cursor.execute('INSERT INTO AvailableBots (namespace) VALUES (?);', (namespace,))
        connection.commit()
        print(f"Added bot with namespace '{namespace}' to the database.")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Add bots to the database.")
    parser.add_argument("--bots", type=int, required=True, help="Number of bots to add.")
    args = parser.parse_args()

    # Validate the number of bots
    if args.bots <= 0:
        print("Number of bots must be greater than 0.")
        return

    # Add bots to the AvailableBots table with dynamically created namespaces
    for i in range(args.bots):
        namespace = f"tb3_{i}"
        add_bot(namespace)

if __name__ == '__main__':
    main()
