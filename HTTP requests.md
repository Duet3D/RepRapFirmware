# List of supported HTTP requests

RepRapFirmware provides an HTTP interface for dealing with client requests.
These requests are tailored for Duet Web Control in the first place but they may be used by third-party applications, too.

Every request except for `rr_connect` returns HTTP status code `401` if the client does not have a valid session.
If no machine password is set, a user session is created whenever an arbitrary HTTP request is made.
Besides, RepRapFirmware may run short on memory and may not be able to respond properly. In this case, HTTP status code `503` is returned.

In the following requests datetime strings may be used. These datetime strings are very similar to ISO8601-encoded strings but they do not contain timezone details.
An example for such a string is `2019-12-13T21:27:00`. Make sure to follow this format where applicable.

## GET /rr_connect

Attempt to create a new connection and log in using the (optional) password.

Supported parameters:

- `password` (optional): Machine password to log in with. If omitted, this defaults to `reprap`
- `time` (optional): Current datetime that will be used to update RepRapFirmware's internal clock

Returns

```
{
    "err": code
}
```

where `code` may be one of the following:

- `0`: Password is valid and the client's IP address is added to the list of HTTP sessions
- `1`: Password is invalid
- `2`: No more user sessions available. This may occur when too many client devices are connected at the same time

If the password is valid, extra fields are returned:

```
{
    "err": 0,
    "sessionTimeout": 8000,
    "boardType": "duetwifi102"
}
```

- `sessionTimeout` is the idle timeout in milliseconds before a client is removed again from the list of client sessions
- `boardType` represents the board's type

Officially supported board types are:

| boardType | Board | Remarks |
| --------- | ----- | ------- |
| duet06 | Duet 0.6 | deprecated |
| duet07 | Duet 0.7 | deprecated |
| duet085 | Duet 0.8.5 | deprecated |
| duetwifi10 | Duet WiFi v1.0 ||
| duetwifi102 | Duet WiFi v1.02 ||
| duetethernet10 | Duet Ethernet v1.0 ||
| duetethernet102 | Duet Ethernet v1.02 ||
| duetmaestro100 | Duet Maestro v1.0 ||
| duet3mb6hc | Duet 3 v0.6 ||

## GET /rr_disconnect

Disconnect again from the RepRapFirmware controller.

Returns

```
{
    "err": code
}
```

where `code` may be `0` if the session could be removed, else it is `1`.

## GET /rr_status

Retrieve a status response from RepRapFirmware in JSON format.

Supported parameters:

- `type`: Type of the status response

The value of `type` can be one of:

- 1: Standard status response
- 2: Advanced status response. This also contains fields from the standard status response
- 3: Print status response. This contains fields from the standard status response as well as information about the current (print) job

See [JSON responses](JSON%20Responses.md) for further information.

## Get /rr_config

Retrieve the configuration response. This request provides a JSON object with values that are expected to change rarely. 

See [JSON responses](JSON%20Responses.md) for further information.

## GET /rr__gcode

Execute arbitrary G/M/T-code(s).

Supported parameters:

- `gcode`: G/M/T-code to execute. This parameter must be present although it can be empty

Returns

```
{
    "buff": bufferSpace
}
```

where `bufferSpace` indicates how much buffer space for new G/M/T-codes is still available.

If a file is supposed to be streamed over the HTTP interface, call this repeatedly and transfer only as many bytes as allowed.

## GET /rr_reply

Retrieve the last G-code reply. The G-code reply is buffered per connected HTTP client and it is discarded when every HTTP client has fetched it or
when the firmware is short on memory and the client has not requested it within reasonable time (1 second).

The G-code reply is returned as `text/plain`.

## GET /rr_upload

Get the last file upload result.

Returns

```
{
    "err": code
}
```

where `code` can be either `0` if the last upload successfully finished or `1` if an error occurred.

## POST /rr_upload

Upload a file.

Supported URL parameters:

- `name`: Path to the file to upload
- `time` (optional): ISO8601-like represenation of the time the file was last modified
- `crc32` (optional): CRC32 checksum of the file content as hex string *without* leading `0x`. Usage of this parameter is encouraged

The raw HTTP body contains the file content. Binary file uploads are supported as well.
Make sure to set `Content-Length` to the length of the HTTP body if your HTTP client does not already do that.

Returns

```
{
    "err": code
}
```

where `code` can be either `0` if the last upload successfully finished or `1` if an error occurred (e.g. CRC mismatch).

## GET /rr_download

Download a file.

Supported parameters:

- `name`: Filename to download

If the file could not be found, HTTP status code 404 is returned, else the file content is sent to the client.

## GET /rr_delete

Delete a file or directory.

Supported parameters:

- `name`: Filename to delete

Returns

```
{
    "err": {code}
}
```

where code is either `0` on success or `1` on error.

## GET /rr_filelist

Retrieve a (partial) file list.

Supported parameters:

- `dir`: Directory to query
- `first` (optional): First item index to return. Defaults to `0`

Returns
```
{
    "dir": dir,
    "first": first,
    "files": [
        {
            "type": itemType,
            "name": itemName,
            "size": itemSize,
            "date": itemDate
        },
        ...
    ],
    "next": next,
    "err": code
}
```

where `dir` and `first` equal the query parameters. `err` can be one of:

- `0`: List query successful
- `1`: Drive is not mounted
- `2`: Directory does not exist

The `next` field may be omitted if the query was not successful and it is `0` if the query has finished. If there are more items to query, this value can be used for the `first` parameter of a successive `rr_filelist` request.

Retrieved files are returned as part of the `files` array. Note that `date` may not be present if it is invalid. Every item has the following properties:

- `itemType`: Type of the file item. This is `d` for directories and `f` for files
- `itemName`: Name of the file or directory
- `itemSize`: Size of the file. This is always `0` for directories
- `itemDate`: Datetime of the last file modification

## GET /rr_files

Retrieve a list of files without any attributes.

Supported parameters:

- `dir`: Directory to query
- `first` (optional): First item index to return. Defaults to `0`
- `flagDirs` (optional): Set this to `1` to prefix directory items with `*`. Defaults to `0`

Returns

```
{
    "dir": dir,
    "first": first,
    "files": [
        "file1",
        "file2",
        ...
    ],
    "next": next,
    "err": err
}
```

where `dir` and `first` equal the query parameters. `err` can be one of:

- `0`: List query successful
- `1`: Drive is not mounted
- `2`: Directory does not exist

The `next` field may be omitted if the query was not successful and it is `0` if the query has finished. If there are more items to query, this value can be used for the `first` parameter of a successive `rr_files` request.

## GET /rr_move

Move a file or directory.

Supported parameters:

- `old`: Current path to the file or directory
- `new`: New path of the file or directory
- `deleteexisting` (optional): Set this to `yes` to delete the new file if it already exists. Defaults to `no`

Returns

```
{
    "err": code
}
```

where code is either `0` on success or `1` on error.

## GET /rr_mkdir

Create a new directory.

Supported parameters:

- `dir`: Path to the new directory

Returns

```
{
    "err": code
|
```

where code is either `0` on success or `1` on error.

## GET /rr_fileinfo

Parse a G-code job file and return retrieved information. If no file is specified, information about the file being printed is returned.

Supported parameters:

- `name` (optional): Path to the file to parse

Returns

```
{
    "err": code,
    "size": size,
    "lastModified": lastModified,
    "height": height,
    "firstLayerHeight": firstLayerHeight,
    "layerHeight": layerHeight,
    "printTime": printTime,
    "simulatedTime": simulatedTime,
    "filament": filament,
    "printDuration": printDuration,
    "fileName": fileName,
    "generatedBy": generatedBy
}
```

where `code` is either `0` on success or `1` on error. If the file could not be parsed, all other fields are omitted.

Other fields are:

| Field | Unit | Description | Default value if invalid | Present if `name` is omitted |
| ----- | ---- | ----------- | ------------------------ | --------------------------- |
| size | bytes | Size of the file in bytes | not applicable | yes |
| lastModified | datetime | Datetime of the last file modification | omitted | maybe |
| height | mm | Final print height of the object | 0.00 | yes |
| firstLayerHeight | mm | Height of the first layer | 0.00 | yes |
| layerHeight | mm | Layer height | 0.00 | yes |
| printTime | seconds | Estimated print time | omitted | maybe |
| simulatedTime | seconds | Estimated print time according to the last simulation | omitted | maybe |
| filament | array of mm | Total filament consumption per extruder | empty array | yes |
| printDuration | seconds | Total print time so far | not applicable | no |
| fileName | file path | Absolute path to the file being printed | not applicable | no |
| generatedBy | text | Slicer or toolchain which generated this file | empty string | yes |
