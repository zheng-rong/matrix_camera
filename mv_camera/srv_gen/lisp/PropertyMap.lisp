; Auto-generated. Do not edit!


(cl:in-package mv_camera-srv)


;//! \htmlinclude PropertyMap-request.msg.html

(cl:defclass <PropertyMap-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (identifier
    :reader identifier
    :initarg :identifier
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass PropertyMap-request (<PropertyMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PropertyMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PropertyMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mv_camera-srv:<PropertyMap-request> is deprecated: use mv_camera-srv:PropertyMap-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <PropertyMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mv_camera-srv:command-val is deprecated.  Use mv_camera-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <PropertyMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mv_camera-srv:identifier-val is deprecated.  Use mv_camera-srv:identifier instead.")
  (identifier m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <PropertyMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mv_camera-srv:value-val is deprecated.  Use mv_camera-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<PropertyMap-request>)))
    "Constants for message type '<PropertyMap-request>"
  '((:GET_PROPERTY_LIST . 0)
    (:SET_PROPERTY . 1)
    (:GET_PROPERTY_INFO . 2)
    (:SEARCH_PROPERTY_MAP . 3)
    (:SAVE_SETTINGS . 4)
    (:LOAD_SETTINGS . 5)
    (:START_CAPTURE_PROCESS . 6)
    (:STOP_CAPTURE_PROCESS . 7)
    (:CAPTURE_SINGLE_FRAME . 8)
    (:RESTART_DEVICE . 9)
    (:CLOSE_DEVICE . 10)
    (:OPEN_DEVICE . 11)
    (:LOAD_PROPERTIES . 12)
    (:SHOW_FLAGS . flagsOn)
    (:SHOW_VALUES . valuesOn)
    (:TESTD . 1.23456))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'PropertyMap-request)))
    "Constants for message type 'PropertyMap-request"
  '((:GET_PROPERTY_LIST . 0)
    (:SET_PROPERTY . 1)
    (:GET_PROPERTY_INFO . 2)
    (:SEARCH_PROPERTY_MAP . 3)
    (:SAVE_SETTINGS . 4)
    (:LOAD_SETTINGS . 5)
    (:START_CAPTURE_PROCESS . 6)
    (:STOP_CAPTURE_PROCESS . 7)
    (:CAPTURE_SINGLE_FRAME . 8)
    (:RESTART_DEVICE . 9)
    (:CLOSE_DEVICE . 10)
    (:OPEN_DEVICE . 11)
    (:LOAD_PROPERTIES . 12)
    (:SHOW_FLAGS . flagsOn)
    (:SHOW_VALUES . valuesOn)
    (:TESTD . 1.23456))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PropertyMap-request>) ostream)
  "Serializes a message object of type '<PropertyMap-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'identifier))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'identifier))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PropertyMap-request>) istream)
  "Deserializes a message object of type '<PropertyMap-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'identifier) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'identifier) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PropertyMap-request>)))
  "Returns string type for a service object of type '<PropertyMap-request>"
  "mv_camera/PropertyMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PropertyMap-request)))
  "Returns string type for a service object of type 'PropertyMap-request"
  "mv_camera/PropertyMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PropertyMap-request>)))
  "Returns md5sum for a message object of type '<PropertyMap-request>"
  "e57467d958b91c16e19833e5c8c4fa20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PropertyMap-request)))
  "Returns md5sum for a message object of type 'PropertyMap-request"
  "e57467d958b91c16e19833e5c8c4fa20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PropertyMap-request>)))
  "Returns full string definition for message of type '<PropertyMap-request>"
  (cl:format cl:nil "int8 GET_PROPERTY_LIST=0~%int8 SET_PROPERTY=1~%int8 GET_PROPERTY_INFO=2~%int8 SEARCH_PROPERTY_MAP=3~%int8 SAVE_SETTINGS=4~%int8 LOAD_SETTINGS=5~%~%int8 START_CAPTURE_PROCESS=6~%int8 STOP_CAPTURE_PROCESS=7~%int8 CAPTURE_SINGLE_FRAME=8~%int8 RESTART_DEVICE=9~%int8 CLOSE_DEVICE=10~%int8 OPEN_DEVICE=11~%int8 LOAD_PROPERTIES=12~%~%string SHOW_FLAGS=flagsOn~%string SHOW_VALUES=valuesOn~%~%int8 command~%string identifier~%string value~%float64 testd=1.23456~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PropertyMap-request)))
  "Returns full string definition for message of type 'PropertyMap-request"
  (cl:format cl:nil "int8 GET_PROPERTY_LIST=0~%int8 SET_PROPERTY=1~%int8 GET_PROPERTY_INFO=2~%int8 SEARCH_PROPERTY_MAP=3~%int8 SAVE_SETTINGS=4~%int8 LOAD_SETTINGS=5~%~%int8 START_CAPTURE_PROCESS=6~%int8 STOP_CAPTURE_PROCESS=7~%int8 CAPTURE_SINGLE_FRAME=8~%int8 RESTART_DEVICE=9~%int8 CLOSE_DEVICE=10~%int8 OPEN_DEVICE=11~%int8 LOAD_PROPERTIES=12~%~%string SHOW_FLAGS=flagsOn~%string SHOW_VALUES=valuesOn~%~%int8 command~%string identifier~%string value~%float64 testd=1.23456~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PropertyMap-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'identifier))
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PropertyMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PropertyMap-request
    (cl:cons ':command (command msg))
    (cl:cons ':identifier (identifier msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude PropertyMap-response.msg.html

(cl:defclass <PropertyMap-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PropertyMap-response (<PropertyMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PropertyMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PropertyMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mv_camera-srv:<PropertyMap-response> is deprecated: use mv_camera-srv:PropertyMap-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <PropertyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mv_camera-srv:result-val is deprecated.  Use mv_camera-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <PropertyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mv_camera-srv:status-val is deprecated.  Use mv_camera-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PropertyMap-response>) ostream)
  "Serializes a message object of type '<PropertyMap-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PropertyMap-response>) istream)
  "Deserializes a message object of type '<PropertyMap-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PropertyMap-response>)))
  "Returns string type for a service object of type '<PropertyMap-response>"
  "mv_camera/PropertyMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PropertyMap-response)))
  "Returns string type for a service object of type 'PropertyMap-response"
  "mv_camera/PropertyMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PropertyMap-response>)))
  "Returns md5sum for a message object of type '<PropertyMap-response>"
  "e57467d958b91c16e19833e5c8c4fa20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PropertyMap-response)))
  "Returns md5sum for a message object of type 'PropertyMap-response"
  "e57467d958b91c16e19833e5c8c4fa20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PropertyMap-response>)))
  "Returns full string definition for message of type '<PropertyMap-response>"
  (cl:format cl:nil "string result~%int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PropertyMap-response)))
  "Returns full string definition for message of type 'PropertyMap-response"
  (cl:format cl:nil "string result~%int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PropertyMap-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PropertyMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PropertyMap-response
    (cl:cons ':result (result msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PropertyMap)))
  'PropertyMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PropertyMap)))
  'PropertyMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PropertyMap)))
  "Returns string type for a service object of type '<PropertyMap>"
  "mv_camera/PropertyMap")