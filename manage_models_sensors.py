#!/bin/env python3

import argparse
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
    """
    Asks for a JSON file to upload as parameters 

    Returns:
        str: JSON string if user uploaded a file
             None if they didn't
    """
    json_str = None

    # Ask for a path to a JSON file
    while not json_str:
        json_in = input(
            'Enter path to JSON file with model parameters (enter to skip): ')

        # If user entered a file name
        if json_in != '':
            try:
                # Open the file and try to parse it as JSON
                # Then we break and return
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


def ask_value(desc: str, required: bool, max_len: int, ask_id=False) -> str:
    """
    Asks for valu to insert / update into the table

    Args:
        desc (str): Field name to ask the user to insert
        required (bool): If True, the user cannot skip entering the value
        max_len (int): Maximum lenght of the entry in characters
        max_len (bool): Maximum lenght of the entry in characters
        ask_id (bool): Changes the input prompt. If True, it asks the user
                   to enter an ID for the table to be connected to the foreign key.
                   If False, it will just ask for a value. Defaults toi False
    Returns:
        str: The string entered by the user. None if user skipped the entry
    """
    entry = None
    skip_str = '' if required else '(enter to skip)'

    while not entry:
        if ask_id:
            entry_prompt = "Enter ID for the chosen row: "
        else:
            entry_prompt = f"Enter value for '{desc}' field, {max_len} chars max {skip_str}: "

        entry = input(entry_prompt)

        if len(entry) > max_len:
            print("ERROR: Entry is too long!")
            entry = None
            
        if ask_id:
            try:
                int(entry)
            except ValueError:
                print("ERROR: ID needs to be an integer!")
                entry = None

        if entry == '' and not required:
            entry = None
            break

    return entry


def get_foreign_key(fk_table: str, table_struct: dict,
                    cur: mysql.connector.cursor.MySQLCursor, print_json: bool) -> int:
    """
    Gets a foreign key from the user

    Args:
        dataBase (mysql.connector.connect): Database connection object
        fk_table (str): Name of the table the foreign key points to
        table_struct (dict): Dictionary object representing the table structure
        cur (mysql.connector.cursor.MySQLCursor): Database cursor object
        print_json (bool): If True, also the JSON data is printed when reading back
                           the data from the table

    Returns:
        int: id of the row in in the foreign key table to be associated with the foreign key
    """

    list_entries(table_struct, fk_table, cur, print_json, id=-1)

    id = int(ask_value("", True, 16, ask_id=True))
    return id


def db_insert(database: mysql.connector.connect, tab_name: str, keys: list,
              values: list, cur: mysql.connector.cursor.MySQLCursor) -> int:
    """
    Inserts a row into the DB

    Args:
        database (mysql.connector.connect): Database connection object
        tab_name (str): Name of the table to insert the row into
        keys (list): List of all keys. Needs to be the same length as values
        values (list): List of all values of the keys to be inserted. 
                       Needs to be the same length as keys
        cur (mysql.connector.cursor.MySQLCursor): Database cursor object

    Returns:
        int: ID of the inserted row
    """

    # Build an SQL query for inserting a row
    insertstr = f"INSERT INTO {tab_name} ("
    for i, name in enumerate(keys):
        sep = ', ' if i < len(keys) - 1 else ') '
        insertstr += f"{name}{sep}"

    insertstr += "VALUES ("
    for i, name in enumerate(keys):
        s = '%s, ' if i < len(keys) - 1 else '%s)'
        insertstr += s

    # Actually insert it
    cur.execute(insertstr, values)

    database.commit()

    # ID of the row we inserted
    return cur.lastrowid

def get_entries(cur: mysql.connector.cursor.MySQLCursor,
                tab_name: str, field: str, id: int) -> list:
    """
    Gets the IDs of entries from table 'tab_name' where the field with name 'field' == 'id'

    Args:
        cur (mysql.connector.cursor.MySQLCursor): Database cursor object
        tab_name (str): Name of the table to search the entries in
        field (str): Field that should contain the id 'id'
        id (int): ID to search in 'field'

    Returns:
        list: List of the IDs of elements in table 'tab_name' where 
        field with name 'field' == 'id'
    """

    # Build an SQL query
    query = f"SELECT id FROM {tab_name} WHERE {field} = {id}"
    
    # Execute query
    cur.execute(query)

    # Fetch the results
    ret = [item[0] for item in cur.fetchall()]
    return ret

def add_entry(dataBase: mysql.connector.connect, table_struct: dict, tab_key: str,
              cur: mysql.connector.cursor.MySQLCursor, print_json: bool) -> int:
    """
    Inserts an entry into the database

    Args:
        dataBase (mysql.connector.connect): Database connection object
        table_struct (dict): Dictionary object representing the table structure
        tab_key (str): Key that points to the current table int the table structure
        cur (mysql.connector.cursor.MySQLCursor): Database cursor object
        print_json (bool): If True, also the JSON data is printed when reading back
                           the data from the table

    Returns:
        int: 0 when no error occurred, 1 otherwise
    """

    # Get table parameters fromt the JSON structure
    table = table_struct.get(tab_key, None)
    if table is None:
        print(f"ERROR: Cannot find table {tab_key} in {DB_STRUCTURE_JSON}!")
        return 1

    # Read table paramters
    tab_fields = table.get('fields', None)
    tab_name = table.get('table_name', None)
    tab_has_json = table.get('has_json', False)

    keys = []
    values = []

    # Error checking
    if tab_fields is None:
        print(f"ERROR: No fields entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    if tab_name is None:
        print(f"ERROR: No name entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    # Add fields in table_struct to our query list
    for field in tab_fields:
        field_name = field.get('name', None)
        if field_name is None:
            print(f"ERROR: No name for a field in table{tab_name} in {DB_STRUCTURE_JSON}!")
            return 1


        field_required = field.get('required', False)
        field_fk = field.get('fk', False)
        field_fk_table = field.get('fk_table', None)
        if field_fk and field_fk_table is None:
            print(f"ERROR: Field {field_name} in {tab_name} marked as foreign key in {DB_STRUCTURE_JSON}, "
                  "but no field_fk_table entry found!")
            return 1

        field_size = field.get('size', None)
        if field_size is None and not field_fk:
            print(f"Warning field size for field {field_name} not specified! Defaulting to 64")
            field_size = 64
            
        # Field is a foreign key
        if field_fk:
            val = get_foreign_key(field_fk_table, table_struct, cur, print_json)

            if val is not None:
                keys.append(field_name)
                values.append(val)
        # Not a FK
        else:
            val = ask_value(field_name, field_required, field_size)
            if val is not None:
                keys.append(field_name)
                values.append(val)

    # If table has a JSON field, we should also ask for a JSON file to upload
    if tab_has_json:
        val = ask_json()
        if val is not None:
            keys.append('params')
            values.append(str(val))

    # Insert the line to the DB. Get back the ID if the inserted row
    id = db_insert(dataBase, tab_name, keys, values, cur)

    #  Read the inserted row back from the DB
    list_entries(table_struct, tab_key, cur, print_json, id)

    return 0

def delete_entry(database: mysql.connector.connect, cur: mysql.connector.cursor.MySQLCursor, 
                 tab_name: str, ids: list) -> None:

    # Build an SQL query
    query = f"DELETE FROM {tab_name} WHERE"

    for i, id in enumerate(ids):
        if i == 0:
            or_str = ''
        else:
            or_str = 'OR '
            
        query += f" {or_str}id = {id}"

    # Execute query
    cur.execute(query)

    database.commit()

def remove_entry(database: mysql.connector.connect, table_struct: dict, tab_key: str,
                 cur: mysql.connector.cursor.MySQLCursor, print_json: bool):
        
    list_entries(table_struct, tab_key, cur, print_json)
    id = int(ask_value("Please enter the ID of the field to delete",
                True, 16, True))

    # If this entry is used for a foreign key in another table,
    # we need to delete that also 
    table = table_struct.get(tab_key, None)
    if table is None:
        print(f"ERROR: Cannot find table {tab_key} in {DB_STRUCTURE_JSON}!")
        return 1

    tab_name = table.get('table_name', None)
    
    check_fk = True
    if tab_name == 'ml_model':
        fk_tab = 'detection_model'
        fk_tab_field = 'ml_model_id'

    elif tab_name == 'dataset':
        fk_tab = 'detection_model'
        fk_tab_field = 'dataset_id'

    elif tab_name == 'sensor_type':
        fk_tab = 'sensor'
        fk_tab_field = 'sensor_type_id'

    else:
        check_fk = False

    if check_fk:
        fk_entries = get_entries(cur, fk_tab, fk_tab_field, id)
        
        if len(fk_entries) > 0:
            print("If you delete this entry, also the following entries "
                    "reffering to this will be deleted:")
            print(f"From table '{fk_tab}'")
            for entry in fk_entries:
                print(fk_entries, entry)
                list_entries(table_struct, fk_tab, cur, print_json, entry)

    ans  = ''
    while ans != 'y' and ans != 'n':
        ans = input("Confirm delete (Y/N): ")
        ans = ans.lower()

    if ans == 'y':
        print('Deleting')
        
        if len(fk_entries) > 0:
            delete_entry(database, cur, fk_tab, fk_entries)

        delete_entry(database, cur, tab_name, [id])


    elif ans == 'n':
        print("Delete cancelled by user")
        exit(0)

def list_entries(table_struct: dict, tab_key: str, cur: mysql.connector.cursor.MySQLCursor, 
                 print_json: bool, id=-1) -> int:
    """
    Reads all table contents and displays on screen. If id == -1,
    it will read the contents of all tables. If id is non-negative,
    it will read the contents of the row with id number defined by the id parameter

    Args:
        table_struct (dict): Dictionary object representing the table structure
        tab_key (str): Key that points to the current table in the table structure
        cur (mysql.connector.cursor.MySQLCursor): Database cursor object
        print_json (bool): If True, also the JSON data is printed when reading back
                           the data from the table
        id (int, optional): ID of the row to read from the table.
                            If id is negative, all rows are read. Defaults to -1.

    Returns:
        int: 0 when no error occurred, one otherwise
    """

    table = table_struct.get(tab_key, None)
    if table is None:
        print(f"ERROR: Cannot find table {tab_key} in {DB_STRUCTURE_JSON}!")
        return 1

    # Read table paramters
    tab_fields = table.get('fields', None)
    tab_name = table.get('table_name', None)
    tab_has_json = table.get('has_json', False)

    # Keys to be read. Other keys will be added later
    keys = ["id"]

    if tab_fields is None:
        print(f"ERROR: No fields entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    if tab_name is None:
        print(f"ERROR: No name entry for table in {DB_STRUCTURE_JSON}!")
        return 1

    # Find all fields that need to be read from the table
    # and add them to the list of keys
    for field in tab_fields:
        field_name = field.get('name', None)

        if field_name is not None:
            keys.append(field_name)

    # If we should also read the JSON parameters, add it to the keys
    if print_json and tab_has_json:
        keys.append('params')

    field_str = ''

    if len(keys) > 0:

        # Create the SQL SELECT query for getting the rows
        for i, name in enumerate(keys):
            s = f'{name}, ' if i < len(keys) - 1 else f'{name} '
            field_str += s

        query = f"SELECT {field_str} FROM {tab_name}"

        if id >= 0:
            query += f" WHERE id = {id}"

        # Execute the DB query
        cur.execute(query)

        rows = cur.fetchall()

        # Print results
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
                    if keys[i] == "params" and field is not None:
                        print("Params:")
                        jsonstr = json.loads(field)
                        print(json.dumps(jsonstr, indent=4))
                    else:
                        print(f"{keys[i]}: {field}")
            print("====================")
            print()

    return 0


def arg_parser() -> argparse.ArgumentParser:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    cmd_group = parser.add_argument_group("Action (choose one)")
    cmd = cmd_group.add_mutually_exclusive_group()
    cmd.add_argument('-i', '--insert', action='store_true',
                     help='Add ML object to database')
    cmd.add_argument('-r', '--remove', action='store_true',
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
    gen_group = parser.add_argument_group("General arguments")
    gen_group.add_argument('-pj', '--print-json', action='store_true',
                           help='If specied, it will cause also the JSON parameters '
                           'to be printed out while listing table contents. '
                           'If this flag is not specified, the JSON parameters '
                           'are ignored while printing.')

    return parser


def main(parser: argparse.ArgumentParser) -> int:

    args = parser.parse_args()

    if not args.insert and not args.remove and not args.list:
        print("ERROR: No command specified (what do you want to do?)!", file=stderr)
        print()
        parser.print_help()
        return 1

    if not args.machine_learning and not args.dataset and not args.detection_model \
        and not args.sensor and not args.sensor_type:
        
        print("ERROR: No object type specified (what table do you want to modify?)!", file=stderr)
        print()
        parser.print_help()
        return 1

    # Read DB structure from JSON file for easier manipulation
    table_struct = None
    try:
        with open(DB_STRUCTURE_JSON, 'r') as f:
            table_struct = json.load(f)
    except (FileNotFoundError, json.decoder.JSONDecodeError) as e:
        print()
        print("ERROR: Cannot read JSON file:", str(e))
        return 1

    if args.machine_learning:
        tab_key = 'ml_model'
    elif args.dataset:
        tab_key = 'dataset'
    elif args.detection_model:
        tab_key = 'detection_model'
    elif args.sensor_type:
        tab_key = 'sensor_type'
    elif args.sensor:
        tab_key = 'sensor'
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
        retval = add_entry(dataBase, table_struct, tab_key,
                           db_cursor, args.print_json)

    elif args.remove:
        retval = remove_entry(dataBase, table_struct, tab_key, 
                              db_cursor, args.print_json)

    elif args.list:
        retval = list_entries(table_struct, tab_key,
                              db_cursor, args.print_json)

    else:
        print()
        print(f"ERROR: No command specified!")
        return 1

    dataBase.close()

    return retval


if __name__ == '__main__':
    parser = arg_parser()
    exit(main(parser))
