#!/usr/bin/env python3
"""Offline FMT-based altitude evidence reader; no MAVLink connection or library."""
import argparse
import json
import mmap
from pathlib import Path
import struct


def altitude_evidence(path):
    formats = {128: (89, 'FMT', 'BBnNZ', 'Type,Length,Name,Format,Columns')}
    codes = {'b': 'b', 'B': 'B', 'M': 'B', 'h': 'h', 'H': 'H', 'c': 'h', 'C': 'H',
             'i': 'i', 'I': 'I', 'e': 'i', 'E': 'I', 'L': 'i', 'f': 'f', 'd': 'd',
             'q': 'q', 'Q': 'Q', 'n': '4s', 'N': '16s', 'Z': '64s', 'a': '32h'}
    result, altitude = [], {}
    with path.open('rb') as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as blob:
        offset = 0
        while offset + 3 <= len(blob):
            if blob[offset:offset+2] != b'\xa3\x95':
                offset = blob.find(b'\xa3\x95', offset+1)
                if offset < 0:
                    break
            kind = blob[offset+2]
            if kind not in formats:
                offset += 3
                continue
            length, name, fmt, columns = formats[kind]
            payload = blob[offset+3:offset+length]
            offset += length
            if len(payload) != length-3:
                break
            if kind == 128:
                type_id, size, raw_name, raw_fmt, raw_columns = struct.unpack('<BB4s16s64s', payload)
                clean = lambda raw: raw.split(b'\0')[0].decode()
                formats[type_id] = (size, clean(raw_name), clean(raw_fmt), clean(raw_columns))
                continue
            if name not in ('MODE', 'ARM', 'CTUN', 'ORGN'):
                continue
            try:
                values = struct.unpack('<' + ''.join(codes[letter] for letter in fmt), payload)
            except (KeyError, struct.error):
                continue
            data = dict(zip(columns.split(','), values))
            if name == 'CTUN':
                altitude = {key: data.get(key) for key in ('TimeUS', 'Alt', 'DAlt')}
            else:
                if name == 'ORGN':
                    data['Alt_m'] = data['Alt'] * .01
                result.append({'type': name, 'data': data, 'preceding_CTUN_m': altitude})
    return {'path': str(path), 'events': result, 'note': 'ORGN Alt scaled cm to m; CTUN Alt/DAlt float metres; other raw fields unscaled'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('path', type=Path)
    args = parser.parse_args()
    print(json.dumps(altitude_evidence(args.path), indent=2))
