import os
import tornado.ioloop
import tornado.web

# root = os.getcwd()
root = os.path.dirname(__file__)
print(root)
# print(root +"/capstone/")
port = 8888

application = tornado.web.Application([
    (r"/(.*)", tornado.web.StaticFileHandler, {"path": root, "default_filename": "index.html"})
])

if __name__ == '__main__':
    application.listen(port)
    tornado.ioloop.IOLoop.instance().start()
