
(cl:in-package :asdf)

(defsystem "mv_camera-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PropertyMap" :depends-on ("_package_PropertyMap"))
    (:file "_package_PropertyMap" :depends-on ("_package"))
  ))