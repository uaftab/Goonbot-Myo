import urllib2
import sys
print(sys.argv, len(sys.argv))
urlstring = "http://192.168.4.1/"+str(sys.argv[1])
urllib2.urlopen(urlstring).read()
