import sqlite3

DB_PATH = 'data/messages.db'

def init_db(path: str = DB_PATH):
    conn = sqlite3.connect(path)
    cur = conn.cursor()
    cur.execute('''
        CREATE TABLE IF NOT EXISTS messages (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            topic TEXT NOT NULL,
            content TEXT NOT NULL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    conn.commit()
    return conn


def create_message(conn: sqlite3.Connection, topic: str, content: str) -> int:
    cur = conn.cursor()
    cur.execute('INSERT INTO messages (topic, content) VALUES (?, ?)', (topic, content))
    conn.commit()
    return cur.lastrowid


def read_messages(conn: sqlite3.Connection, **filters) -> list:
    sql = 'SELECT * FROM messages'
    params = []
    if filters:
        clauses = []
        for k, v in filters.items():
            clauses.append(f"{k}=?")
            params.append(v)
        sql += ' WHERE ' + ' AND '.join(clauses)
    cur = conn.cursor()
    cur.execute(sql, params)
    return cur.fetchall()


def update_message(conn: sqlite3.Connection, message_id: int, content: str) -> int:
    cur = conn.cursor()
    cur.execute('UPDATE messages SET content=? WHERE id=?', (content, message_id))
    conn.commit()
    return cur.rowcount


def delete_message(conn: sqlite3.Connection, message_id: int) -> int:
    cur = conn.cursor()
    cur.execute('DELETE FROM messages WHERE id=?', (message_id,))
    conn.commit()
    return cur.rowcount