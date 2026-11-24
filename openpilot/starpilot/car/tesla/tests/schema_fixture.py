"""Compare complete compiled layouts, including opaque pointer default values."""
def layout(schema):
  result = schema.node.struct.to_dict()
  for field in result.get('fields', []):
    if 'slot' in field and next(iter(field['slot']['type'])) in ('struct', 'list', 'anyPointer', 'data'):
      value = schema.fields[field['name']].proto.slot.defaultValue
      field['slot']['defaultValue'] = {'schemaValueHex': value.as_builder().to_bytes().hex()}
  return result
