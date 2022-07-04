#!/bin/env python3

import argparse
from dataclasses import fields
from sys import stderr
import json
from enum import Enum
import mysql.connector

DB = 'transport_ecosystem_management_db'
DB_USER = 'db_user'
DB_PASS = 'transport123'
DB_HOST = '172.17.0.1'
# DB_HOST = 'host.docker.internal'
DB_PORT = 3306

DB_STRUCTURE_JSON = "structure.json"


class Actions(Enum):
    ADD = 1
    REMOVE = 2
    EDIT = 3


def ask_json() -> str:
    json_str = None

    while not json_str:
        json_in = input(
            'Enter path to JSON file with model parameters (enter to skip): ')

        if json_in != '':
            try:
                with open(json_in, 'r') as f:
                    j_data = json.load(f)
                    json_str = json.dumps(j_data)
                    break
            except (FileNotFoundError, json.decoder.JSONDecodeError) as e:
                print()
                print("ERROR: Cannot read JSON file:", str(e))
        else:
            break
    return json_str


def ask_value(desc: str, required: bool, max_len: int) -> str:
    entry = None
    skip_str = '' if required else '(enter to skip)'

    while not entry:
        entry = input(
            f"Enter value for '{desc}' field, {max_len} chars max {skip_str}: ")
        if len(entry) > max_len:
            print("ERROR: Entry is too long!")
            entry = None

        if entry == '' and not required:
            entry = None
            break

    return entry


def get_foreign_key(fk_table: str) -> int:
    print("Foreign key STUB")
    return 0


def db_insert(database: mysql.connector.connect, tab_name: str, names: list,
              values: list, cur: mysql.connector.cursor.MySQLCursor):

    insertstr = f"INSERT INTO {tab_name} ("
    for i, name in enumerate(names):
        sep = ', ' if i < len(names) - 1 else ') '
        insertstr += f"{name}{sep}"

    insertstr += "VALUES ("
    for i, name in enumerate(names):
        s = '%s, ' if i < len(names) - 1 else '%s)'
        insertstr += s

    print(insertstr)
    print(values)

    cur.execute(insertstr, values)
    database.commit()


def add_entry(dataBase: mysql.connector.connect, table_struct: dict,
              cur: mysql.connector.cursor.MySQLCursor):

    tab_fields = table_struct.get('fields', None)
    tab_name = table_struct.get('table_name', None)
    tab_has_json = table_struct.get('has_json', False)

    names = []
    values = []

    if tab_fields is None:
        print(f"ERROR: No fields entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    if tab_name is None:
        print(f"ERROR: No name entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    for field in tab_fields:
        field_name = field.get('name', None)
        if field_name is None:
            print(
                f"ERROR: No name for a field in table{tab_name} in {DB_STRUCTURE_JSON}!")
            return 1

        field_size = field.get('size', None)
        if field_size is None:
            print(
                f"Warning field size for field {field_name} not specified! Defaulting to 64")
            field_size = 64

        field_required = field.get('required', False)
        field_fk = field.get('fk', False)
        field_fk_table = field.get('fk_table', None)
        if field_fk and field_fk_table is None:
            print(f"ERROR: Field {field_name} in {tab_name} marked as foreign key in {DB_STRUCTURE_JSON}, "
                  "but no field_fk_table entry found!")
            return 1

        if field_fk:
            val = get_foreign_key(field_fk_table)
            if val is not None:
                names.append(field_name)
                values.append(val)
        else:
            val = ask_value(field_name, field_required, field_size)
            if val is not None:
                names.append(field_name)
                values.append(val)

    if tab_has_json:
        val = ask_json()
        if val is not None:
            names.append('params')
            values.append(str(val))

    db_insert(dataBase, tab_name, names, values, cur)

    return 0


def remove_entry(dataBase: mysql.connector.connect, table_struct: dict,
                 cur: mysql.connector.cursor.MySQLCursor):
    pass


def modify_entry(dataBase: mysql.connector.connect, table_struct: dict,
                 cur: mysql.connector.cursor.MySQLCursor):
    pass

def list_entries(dataBase: mysql.connector.connect, table_struct: dict,
                 cur: mysql.connector.cursor.MySQLCursor):

    tab_fields = table_struct.get('fields', None)
    tab_name = table_struct.get('table_name', None)
    tab_has_json = table_struct.get('has_json', False)

    names = ["id"]

    if tab_fields is None:
        print(f"ERROR: No fields entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    if tab_name is None:
        print(f"ERROR: No name entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    for field in tab_fields:
        field_name = field.get('name', None)


        if field_name is not None:
            names.append(field_name)


    field_str = ''

    if len(names) > 0:
        for i, name in enumerate(names):
            s = f'{name}, ' if i < len(names) - 1 else f'{name} '
            field_str += s

        query = f"SELECT {field_str} FROM {tab_name}"
        cur.execute(query)
        
        rows = cur.fetchall()

        print(names)
        print(rows)

        print()
        print("Listing entries:")
        print("====================")
        print(f"Table: {tab_name}")
        print("====================")
        print()
        for row in rows:
            print("====================")
            for i, field in enumerate(row):
                if i == 0:
                    print(f"ID: {field}")
                    print("--------------------")
                else:
                    print(f"{names[i]}: {field}")
            print("====================")
            print()



def arg_parser() -> argparse.ArgumentParser:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    cmd_group = parser.add_argument_group("Action (choose one)")
    cmd = cmd_group.add_mutually_exclusive_group()
    cmd.add_argument('-i', '--insert', action='store_true',
                     help='Add ML object to database')
    cmd.add_argument('-r', '--remove', action='store_true',
                     help='Remove ML object to database')
    cmd.add_argument('-u', '--update', action='store_true',
                     help='Remove ML object to database')
    cmd.add_argument('-l', '--list', action='store_true',
                     help='Lists the entries in a table')

    type_group = parser.add_argument_group("Object to modify (choose one)")
    type = type_group.add_mutually_exclusive_group()
    type.add_argument('-ml', '--machine-learning', action='store_true',
                      help='Modify machine learning model table')
    type.add_argument('-ds', '--dataset', action='store_true',
                      help='Modify dataset table')
    type.add_argument('-dm', '--detection-model', action='store_true',
                      help='Modify detection model table')
    type.add_argument('-st', '--sensor-type', action='store_true',
                      help='Modify sensor type table')
    type.add_argument('-s', '--sensor', action='store_true',
                      help='Modify sensor table')

    return parser


def main(parser: argparse.ArgumentParser) -> int:

    args = parser.parse_args()

    if not args.insert and not args.remove and not args.update and not args.list:
        print("ERROR: No command specified (what do you want to do?)!", file=stderr)
        print()
        parser.print_help()
        return 1

    if not args.machine_learning and not args.dataset and not args.detection_model:
        print("ERROR: No object type specified (what table do you want to modify?)!", file=stderr)
        print()
        parser.print_help()
        return 1

    # Read DB structure from JSON file for easier manipulation
    j_data = None
    try:
        with open(DB_STRUCTURE_JSON, 'r') as f:
            j_data = json.load(f)
    except (FileNotFoundError, json.decoder.JSONDecodeError) as e:
        print()
        print("ERROR: Cannot read JSON file:", str(e))
        return 1

    table_struct = None

    if args.machine_learning:
        table_struct = j_data.get('ml', None)
    elif args.dataset:
        table_struct = j_data.get('dataset', None)
    elif args.detection_model:
        table_struct = j_data.get('dm', None)
    elif args.sensor_type:
        table_struct = j_data.get('sensor_type', None)
    elif args.sensor:
        table_struct = j_data.get('sensor', None)
    else:
        print()
        print("ERROR: Unknown table!")
        return 1

    if table_struct is None:
        print()
        print(f"ERROR: No entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    dataBase = mysql.connector.connect(
        host=DB_HOST,
        user=DB_USER,
        passwd=DB_PASS,
        port=DB_PORT,
        database=DB

    )

    db_cursor = dataBase.cursor()
    retval = 0

    if args.insert:
        retval = add_entry(dataBase, table_struct, db_cursor)

    elif args.remove:
        retval = remove_entry(dataBase, table_struct, db_cursor)

    elif args.update:
        retval = modify_entry(dataBase, table_struct, db_cursor)

    elif args.list:
        retval = list_entries(dataBase, table_struct, db_cursor)

    else:
        print()
        print(f"ERROR: No command specified!")
        return 1

    dataBase.close()

    return retval


if __name__ == '__main__':
    parser = arg_parser()
    exit(main(parser))
