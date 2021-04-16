#ifndef html_h
#define html_h
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>Rancilio Local</title>
<link rel='icon' href='data:image/png;base64,AAABAAMAMDAAAAEAGACoHAAANgAAACAgAAABABgAqAwAAN4cAAAQEAAAAQAIAGgFAACGKQAAKAAAADAAAABgAAAAAQAYAAAAAAAAGwAAAAAAAAAAAAAAAAAAAAAAAP////////////////////////////////////////7+/v////////////////////////////7///7+/v7+/v7+/v////////7+/v7+/v7+/v7+/v////////////////////////////////////7+/v7+/v////////////////////7+/v/+/v////////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v////n5+ebm5s7Ozrm7u7K0tLm7vM/Q0Onp6fv7+/////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v39/f7+/vX19c7OzrS3tsjLyvHy8v/+/v7+/v7+/v////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///+7u7rKys2xtbj5AQCkrKyAiIx0gISAiIykrLEFCQ3Bycri4ufHx8f////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v39/f///+Pj425vcCorLB4gIScqKmRlZtzc3P/+/v7+/v////7+/v7+/v7+/v7+/v7+/v////n5+bq6ulVVViEiIxcZGhkbHBocHRsdHhsdHhsdHhocHRkbHBcaGyMkJVxcXcLCwvv7+/////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v///uLj415eYBkaHBocHRsdHhocHRcZGmVmZvPz8/////////7+/v7+/v7+/v7+/v////Ly8ouMjCgqKxgaGxscHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhocHhgZGywuL5SWlvX19f///v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v39/f7//uHj411fXxkbHRsdHhsdHhsdHhsdHhocHSkqKszMzP////////7+/v7+/v7+/v////Hx8nl6ex4fIBocHRsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhocHSAiI4WGh/X29v////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v///+Li4lxeXxkbHBsdHhsdHhsdHhsdHhsdHhsdHiAhIrm6uv////////7+/v7+/v7+/vr6+oqJix0fIBocHRsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhocHSAiI5WWl/z8/P7+/v7+/v7+/v7+/v7+/v7+/v39/f///+Pj415eXhkbHBsdHhsdHhsdHhsdHhsdHhsdHhocHSstLs/R0f////////7+/v7+/v///7i4uCkoKhocHRsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhodHRsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhkbHC4vL8PDw/////7+/v7+/v7+/v7+/v39/f7//+Hi41xeXhkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhkbHHN0dff39/7+/v////7+/v7+/uzr7FJSUxkaGxsdHhsdHhsdHhsdHhsdHhsdHhocHRkaGyAhIiQmJx8gIhkaGxocHRsdHhsdHhsdHhsdHhsdHhsdHhkaG15fX/Ly8v7+/v7+/v7+/v3+/v7+/+Hi41xeXxkbHBsdHhsdHhsdHhsdHhsdHhsdHhscHhkbHWJjZOXl5v7+/v7+/v////3+/v///6+uryAhIhsdHhsdHhsdHhsdHhsdHhsdHhkbHDI0NX9/gLe2uMXFxrW0tXt7ezAxMhkbHBsdHhsdHhsdHhsdHhsdHhocHSMlJrq7u/////7+/v39/f///+Di4ltdXhkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhobHmNkZuXm5v////7+/v7+/v////7+/vn5+WloaRgZGxsdHhsdHhsdHhsdHhsdHhkbHEhKSsrKy/38/f////////////z8/MTExENERBkbHBsdHhsdHhsdHhsdHhsdHhgaG3V1dvv7+/7+/v///+Li4lxdXhkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhocHWRmZ+Xn6P7///7+/v7+/v7+/v///////+Tk5Dw8PRkbHBsdHhsdHhsdHhsdHhocHTEyM8jIyP////7+/v7+/v7+/v7+/v7+/v///8LCwi0vLxocHRsdHhsdHhsdHhsdHhkbHEVFRurp6////+Li4l1dXRkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhsbHWZmZ+fn6P7///3+/v7+/v7+/v7+/v///////8rLyyYoKRocHRsdHhsdHhsdHhsdHhgaG3l7fP39/f7+/v7+/v7+/v7+/v7+/v7+/v7+/vr6+nJzcxgaGxsdHhsdHhsdHhsdHhocHS0uL9bV1+Pi5FxcXRkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhocHWVmZ+fn5/////39/f7+/v7+/v7+/v7+/v///////7q7uyAiIxsdHhsdHhsdHhsdHhsdHh4gIbKztP////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///6qqqxweHxsdHhsdHhsdHhsdHhocHSQmJ66ur2FgYhoaHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhocHWZoaebo6P7+/v7+/v7+/v7+/v7+/v7+/v7+/v///////7OztB4gIRsdHhsdHhsdHhsdHhsdHiIkJcDCwv////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///7m6uiAiIxsdHhsdHhsdHhsdHhsdHiIkJVZYWR8gIRwcHhsdHhsdHhsdHhsdHhsdHhsdHhscHWdnaefo6f7///7+/v7+/v7+/v7+/v7+/v7+/v7+/v///////7y9vSAiIxsdHhsdHhsdHhsdHhsdHh0fIKutrf////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///7q6uyAiIxsdHhsdHhsdHhsdHhsdHh4gISYoKRsdHhsdHhsdHhsdHhsdHhsdHhsdHhweH2lqbOjo6f39/v39/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v///////8/Q0CkrLBocHRsdHhsdHhsdHhsdHhgaG21vcPj4+P39/f7+/v7+/v7+/v7+/v7+/v7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh4gISkrLBsdHhsdHhsdHhsdHhsdHhsdHhsdHiEjJKyur/////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v///////+rq60JERRgaGxsdHhsdHhsdHhsdHhocHSkrLLi4uP////39/f7+/v7+/v7+/v7+/v7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHiIkJWBiYiEjJBsdHhsdHhsdHhsdHhsdHhsdHhkbHEJERdbX2P////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v////7+/vz9/XV2dxgaGhsdHhsdHhsdHhsdHhsdHhkbG0JCQ9fX1/////7+/v7+/v7+/v7+/v7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHiAiI6mqq2prbBkbGxsdHhsdHhsdHhsdHhsdHhsdHhkaG15eYOvr7P/+//79/v7+/v7+/v7+/v7+/v7+/v7+/v////7+/v///7y8vCUmJxocHRsdHhsdHhsdHhsdHhsdHhkaG15eX+vr6/////7+/v7+/v7+/v7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIre5uuDg4EhJShgbHBsdHhsdHhsdHhsdHhsdHhscHh0cHn9+gPb19/38/v7+/v7+/v7+/v7+/v7+/v7+/v////7+/v7+/vPz82BhYhcZGhsdHhsdHhsdHhsdHhsdHhsdHhwdHYGBgfj4+P7+/v7+/v7+/v7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP///8LDwzAzNBkbHBsdHhsdHhsdHhsdHhsdHhscHSQjJaKhov38/f7+/v7+/v7+/v7+/v7+/v7+/v////7+/v7+/v///8bHyC8xMhkbHBsdHhsdHhsdHhsdHhsdHhocHSQlJaOkpP7+/v7+/v7+/v7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////3+/qCioiIkJRscHhsdHhsdHhsdHhsdHhsdHhobHDIxM8LCw/////39/f7+/v7+/v7+/v7+/v////7+/v7+/v7+/v39/ZubnCAiIxocHRsdHhsdHhsdHhsdHhsdHhkbHDIzNMLExP////39/f7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/vf3931/gBsdHhsdHhsdHhsdHhsdHhsdHhsdHhkaG0hHSdvb2/////7+/v7+/v7+/v7+/v////7+/v7+/v7+/v7+/vX19Xd5ehocHRsdHhsdHhsdHhsdHhsdHhsdHhgaG0lKS97e3v////7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v///+nq61tdXhgaGxsdHhsdHhsdHhsdHhsdHhsdHhkaG2VlZe7u7v////7+/v7+/v7+/v////7+/v7+/v7+/v7+/v///+bo6FdZWhgaGxsdHhsdHhsdHhsdHhsdHhsdHhobHGhoafDw8P7+/v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7//9TW1j9BQhkbHBsdHhsdHhsdHhsdHhsdHhsdHR0eHoiIiPn5+f7+/v7+/v7+/v////7+/v7+/v7+/v7+/v7+/v7//9LT0z8/QBkbHBsdHhsdHhsdHhsdHhsdHhsdHh0fIIuLjPr6+v///7m6uyAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7+/v///7m5uiwtLhkcHRsdHhsdHhsdHhsdHhsdHhocHScpKba3uP////7+/v7+/v////7+/v7+/v7+/v7+/v7+/v7+/v///7e3tystLRocHRsdHhsdHhsdHhsdHhsdHhocHScoKa2urv///7i5uiAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7+/v7+/vz8/JeYmB8hIhocHRsdHhsdHhsdHhsdHhsdHhcZG1FSU+zs7P7+/v7+/v////7+/v7+/v7+/v7+/v7+/v7+/v7+/vv8/JSVlh4gIhocHRsdHhsdHhsdHhsdHhsdHhkaGzg5Os/Q0bq8vSAiIxsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7+/v7+/v////P09XJ0dRkbHBsdHhsdHhsdHhsdHhsdHhsdHiAhIrCxsf////7+/v////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/vPz9HJzdBocHRsdHhsdHhsdHhsdHhsdHhsdHhgaG1ZYWaKkpSEjJBsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7+/v7+/v7+/v7//+Tl5VNUVBkaGxsdHhsdHhsdHhsdHhsdHhgZGmxsbfr5+v7+/v////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v///+Xm5lRWVxocHRsdHhsdHhsdHhsdHhsdHhsdHh4gIVZYWSIkJRsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7+/v7+/v7+/v7+/v///8rKyjIzMxkbHBsdHhsdHhsdHhsdHhkbHD8+QObl5/////////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v39/f///66vsCEjJBsdHhsdHhsdHhsdHhsdHhsdHhsdHiYoKR4gIRsdHhsdHhsdHhsdHhsdHh8hIra3uP////7+/v7+/v7+/v7+/v7+/v7+/v7+/vz8/Ht8fBgaGxsdHhsdHhsdHhsdHhocHSkpK8zMzf////////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v///9TU1UxOThocHRsdHhsdHhsdHhsdHhsdHhsdHhocHS0vMB8hIhsdHhsdHhsdHhsdHhsdHh8hIra4uP////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///7Gysx4gIRsdHhsdHhsdHhsdHhsdHiIiJLy8vf////////7+/v7+/v7+/v7+/v7+/v7+/v39/f///9TT1UtLTBgaGxsdHhsdHhsdHhsdHhsdHhsdHhocHSMlJmttbiMlJhsdHhsdHhsdHhsdHhsdHh4gIbS1tv////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///8LDwyMlJhsdHhsdHhsdHhsdHhsdHh8hIra3uP////////7+/v7+/v7+/v7+/v7+/v7+/v///9LU1EpLTBkaGxsdHhsdHhsdHhsdHhsdHhsdHhocHR0fH36AgLm7vCQmJxocHRsdHhsdHhsdHhsdHhsdHqChov////7+/v7+/v7+/v7+/v7+/v7+/v7+/v///6ioqRweHxsdHhsdHhsdHhsdHhsdHiEjJL2+v/////////7+/v7+/v7+/v7+/v7+/v///9LT00lLSxgaGxsdHhsdHhsdHhsdHhsdHhsdHhocHR4fH3t8fPPz89nZ2jAxMhocHRsdHhsdHhsdHhsdHhgaG2NkZPX19f7+/v7+/v7+/v7+/v7+/v7+/v7+/vf392lqahgaGxsdHhsdHhsdHhsdHhocHSstLtDR0v////////7+/v7+/v7+/v39/v///9HT00hKShgaGxsdHhsdHhsdHhsdHhsdHhsdHhocHR0fIHp8fPDx8f///+/v701NTRkaGxsdHhsdHhsdHhsdHhscHSYmJ6qqqv7+/v7+/v7+/v7+/v7+/v7+/v7+/q+vrygoKRocHRsdHhsdHhsdHhsdHhkaG0ZGR+vr6/////////7+/v7+/v7+/v///9HS0kdJSRgaGxsdHhsdHhsdHhsdHhsdHhsdHhocHR0fIHp8fO/x8f7+/v7+/v7+/oKCghgaGxsdHhsdHhsdHhsdHhsdHhsbHDIyMqSkpPDx8f////////////Hy8qeoqDQ0NBobHBsdHhsdHhsdHhsdHhsdHhgaG3l4evz8/P/+/v////7+/v7+/v///9HR0khISRgaGxsdHhsdHhsdHhsdHhsdHhsdHhocHR4fIHx8ffDx8f3//v3+/v7+/v///8fHyCorLBocHRsdHhsdHhsdHhsdHhsdHhobGyIjJFRWV4yOj6GjpI2PkVZYWiIkJRsbHBwdHhsdHhsdHhsdHhsdHhocHSYoKcHAwv////7+/v////3+/v/+/9LR00dHSRkaGxsdHhsdHhsdHhsdHhsdHhsdHhocHR0fIHx9fvHx8v7+/v7+/v7+/v7+/v7+/vf2929wcRgaGxsdHhsdHhsdHhsdHhsdHhsdHhsdHRgaGxkbHBocHRkaHBgaGxocHhsdHhsdHhsdHhsdHhsdHhsdHhgaG2doafX09f7+/v7+/v////////Dv8FtaXBgZGhsdHhsdHhsdHhsdHhsdHhsdHhocHR0fIHt9fvDy8v7+/v7+/v7+/v7+/v7+/v7+/v///9PV1To8PRkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhkbHDU3OM7Pz/////7+/v7+/v///////8fHxyYlJxscHRsdHhsdHhsdHhsdHhsdHhocHh4fIXt8fvDy8v3///7+/v7+/v7+/v7+/v7+/v7+/v7+/v3+/6+xsSkrLBkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhwdHhobHCYnKaeoqf39/f7+/v7+/v7+/v///////7i3uR8gIRsdHhsdHhsdHhsdHhsdHhocHR0fIH19f/Ly8v7+/v3+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v3+/vv7+6KjpCssLRgaGxsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhkaHCkoKpubnPn6+v7+/v7+/v7+/v7+/v///////9DQ0SwuLxocHRsdHhsdHhsdHhocHR0fIHt9ffDx8f////7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/v7+/vz8/LS1tkBCQxkbHBkbHBsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhsdHhkcHRgaGzw+P6+vsPr6+v7+/v7+/v7+/v7+/v7+/v////////f3+Hd5ehkbHBkb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9/fy8/NMTk8YGhsbHR4aHB0kJifHyMj////////j5OVQUlMYGhsbHR4bHR4bHR4eHyCQkZH8/Pz+/v7+/v7+/v7+/v7+/v7////c3NxGR0gYGhsbHR4bHR4aHB0hIiSdnp7x8fFNTk8YGhsbHR4aHB0kJifHyMj////+/v7////MzM03OToZGxwbHR4bHR4ZGxwuMDHLy8z////+/v7+/v7+/v7+/v7+/v7////BwcIwMjIZGxwbHR4bHR4ZGxwvMDGys7ROUFEYGhsbHR4aHB0kJifHyMj////+/v7+/v7///+sra4mKCkaHB0bHR4bHR4YGht5env9/f3////+/v7+/v7+/v7+/v7+/v7+/v6jpKQkJicaHB0bHR4bHR4ZGxxCREU7PT4ZGxwbHR4aHB0kJifHyMj////+/v7+/v7+/v77+/uGhoYbHR4bHR4bHR4ZGxxCQkTq6er////+/v7+/v7+/v7+/v7+/v7////c3N08Pj8ZGxwbHR4bHR4bHR4eICEiJCUbHR4bHR4aHB0kJifHyMj////+/v7+/v7+/v7////e3t43OToZGxwbHR4aHB0rLC3S0tP////+/v7+/v7+/v7+/v7////k4+ReX2AbHR4bHR4bHR4bHR4YGhs2OTk2ODkZGxwbHR4aHB0kJifGx8f////+/v7+/v7+/v7////z9PRPUVIYGhsbHR4aHB0lJijIycn////+/v7+/v7+/v7////i4+NdXl8ZGxwbHR4bHR4bHR4ZGxwwMjKsrq5XWVoYGhsbHR4bHR4eICGur7D////+/v7+/v7+/v7////l5uY9P0AZGxwbHR4aHB0oKivOz8/////+/v7+/v7////i4+NcXl4ZGxwbHR4bHR4bHR4ZGxwvMTKys7P+/v54eHkYGRobHR4bHR4ZGhtdXV7t7e3////////////9/f2ZmZoeICEbHR4bHR4ZGxw7PD3l5eX////+/v7////i4uJbXV0ZGxwbHR4bHR4bHR4ZGxwwMTKxs7P9/v7///+xsbEfISIbHR4bHR4bHR4eHx9paWnKy8vo6erb3NyRkpIsLC0aHB0bHR4bHR4XGRpsbG76+vr////////k4+RcXF0ZGxwbHR4bHR4bHR4ZGxwwMjOzs7T+/v7+/v7////r6+tOT1AYGhsbHR4bHR4bHR0ZGxwpKyw+QEEzNDYc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' type='image/x-png' />
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/css/bootstrap.min.css"
    crossorigin="anonymous">
  <script src="https://cdn.jsdelivr.net/npm/vue/dist/vue.js"></script>
  <script src="https://code.jquery.com/jquery-3.4.1.slim.min.js" crossorigin="anonymous"></script>
  <script src="https://cdn.jsdelivr.net/npm/popper.js@1.16.0/dist/umd/popper.min.js" crossorigin="anonymous"></script>
  <script src="https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/js/bootstrap.min.js" crossorigin="anonymous"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/axios/0.19.0/axios.min.js" crossorigin="anonymous"></script>
</head>
<body>
  <div class="container-fluid" id="app">
    <div class="row">
      <div class="col-md-12 mt-3 text-center">
        <h2>Rancilio Firmware Updater</h2>
      </div>
    </div>
    <div class="row mt-4">
      <div class="col-md-12 d-flex justify-content-center">
        <form enctype="multipart/form-data" novalidate class="d-flex flex-column justify-content-center" v-if="!uploadInProgress">
          <div class="alert alert-danger" role="alert" v-if="didError">
            Upload Failed.
          </div>
          <div class="alert alert-success" role="alert" v-if="didSuccess">
            Upload Successful! Device is now restarting.
          </div>
          <div class="custom-file" style="max-width: 400px;" v-if="!didSuccess">
            <input type="file" class="custom-file-input" id="firmwareFile" accept='.bin,.bin.gz' name='firmware' @change="filesChange($event.target.name, $event.target.files);">
            <label class="custom-file-label" for="firmwareFile">{{ filename || 'Choose .bin file' }}</label>
          </div>
          <button type="button" class="btn btn-success mt-3" @click="upload" :disabled="!file" v-if="!didSuccess">Upload</button>
        </form>
        <div v-if="uploadInProgress" class="text-center" style="width:400px;">
          Upload {{ percentCompleted }}% Complete...
          <div class="progress mt-2">
            <div class="progress-bar" role="progressbar" :style="{'width': percentCompleted + '%'}" aria-valuenow="25" aria-valuemin="0"
              aria-valuemax="100"></div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <script>
    var app = new Vue({
      el: '#app',
      data: {
        message: 'Hello Vue!',
        moving: false,
        filename: null,
        file: null,
        percentCompleted: 0,
        uploadInProgress: false,
        didError: false,
        didSuccess: false,
      },
      methods: {
        filesChange: function (filename, files) {
          this.filename = files[0].name;
          this.file = files[0];
        },
        upload: function () {
          this.uploadInProgress = true;
          const formData = new FormData();
          formData.append('firmware', this.file);
          axios.post('/firmware', formData, {
            headers: {
              'Content-Type': 'multipart/form-data'
            },
            onUploadProgress: (progressEvent) => {
              this.percentCompleted = Math.round((progressEvent.loaded * 100) / progressEvent.total)
            }
          })
          .then(() => {
            this.didSuccess = true;
          })
          .catch(() => {
            this.didError = true;
          })
          .finally(() => {
            this.uploadInProgress = false;
          });
        }
      }
    });
  </script>
</body>
</html>
)=====";
#endif