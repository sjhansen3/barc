
(cl:in-package :asdf)

(defsystem "project_231-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))