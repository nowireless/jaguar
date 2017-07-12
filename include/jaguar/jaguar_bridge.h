#ifndef JAGUAR_BRIDGE_H_
#define JAGUAR_BRIDGE_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <map>
#include <vector>
#include <stdint.h>
#include "can_bridge.h"
#include "jaguar_helper.h"

typedef boost::asio::buffers_iterator<
    boost::asio::streambuf::const_buffers_type> asio_iterator;


namespace can {

enum ReceiveState {
    kWaiting,
    kLength,
    kPayload,
    kComplete
};

template<typename T>
class masked_number
{
public:
	typedef T type;

	masked_number(T const& num_, T const& mask_)
	: num(num_), mask(mask_)
	{
		assert((num_ & ~mask_) == 0);
	}

	template <class U>
	masked_number(masked_number<U> const &mn)
	: num(mn.num), mask(mn.mask)
	{}

	bool matches(T val)
	{
		return (val & mask) == num;
	}

	T num;
	T mask;
};

template<typename T>
masked_number<T> make_masked_number(T const &n, T const &m)
{
	return masked_number<T>(n, m);
}

class JaguarToken;

class JaguarBridge : public CANBridge
{
public:
    JaguarBridge(std::string port);
    virtual ~JaguarBridge(void);

    /* a transaction with a response */
    virtual TokenPtr transaction   (CANMessage const &msg, uint32_t resp_id);
    /* a transaction with _no_ response  */
    virtual void     transaction   (CANMessage const &msg);

    /* a transaction with a response which begins a periodic message */
    virtual std::pair<CallbackToken, TokenPtr> start_periodic(CANMessage const &msg, uint32_t resp_id, recv_callback cb, uint32_t cb_id);
    /* a transaction with _no_ response which begins a periodic message */
    virtual CallbackToken start_periodic(CANMessage const &msg, recv_callback cb, uint32_t cb_id);

    /* process errors recived asynchronously */
    virtual CallbackToken attach_callback(error_callback cb);

    virtual CallbackToken attach_callback(recv_callback cb, uint32_t id);
    virtual CallbackToken attach_callback(recv_callback cb, uint32_t id, uint32_t id_mask);

    /* for very special cases */
    virtual TokenPtr recv_only(uint32_t id);
private:

    void send(CANMessage const &message);

    typedef boost::signals2::signal<recv_callback_sig> callback_signal;
    typedef boost::shared_ptr<callback_signal> callback_signal_ptr;

    typedef std::map<uint32_t, callback_signal_ptr> callback_table;

    typedef std::pair<masked_number<uint32_t>, callback_signal_ptr> mask_callback;
    typedef std::list<mask_callback> callback_list;

    typedef boost::shared_ptr<JaguarToken> token_ptr;
    typedef std::map<uint32_t, boost::weak_ptr<JaguarToken> >  token_table;

    static uint8_t const kSOF, kESC;
    static uint8_t const kSOFESC, kESCESC;
    static size_t const kReceiveBufferLength;

    boost::asio::io_service  io_;
    boost::asio::serial_port serial_;

    boost::signals2::signal<error_callback_sig> error_signal_;

    boost::thread recv_thread_;
    std::vector<uint8_t> recv_buffer_;
    callback_table callbacks_;
    callback_list  callbacks_list_;
    boost::mutex callback_mutex_;

    token_table tokens_;
    boost::recursive_mutex token_mutex_;

    std::vector<uint8_t> packet_;
    ReceiveState state_;
    size_t length_;
    bool escape_;

    boost::shared_ptr<CANMessage> recv_byte(uint8_t byte);
    void recv_handle(boost::system::error_code const& error, size_t count);
    void recv_message(boost::shared_ptr<CANMessage> msg);
    void remove_token(boost::shared_ptr<CANMessage> msg);
    void discard_token(JaguarToken &msg);

    boost::shared_ptr<CANMessage> unpack_packet(std::vector<uint8_t> const &packet);
    size_t encode_bytes(uint8_t const *bytes, size_t length, std::vector<uint8_t> &buffer);

    friend class JaguarToken;
};

class JaguarToken : public Token {
public:
    virtual ~JaguarToken(void);
    virtual void block(void);
    virtual bool timed_block(boost::posix_time::time_duration const &duration);
    virtual boost::shared_ptr<CANMessage const> message(void) const;
    virtual bool ready(void) const;
    virtual void discard(void);

private:
    bool done_;
    JaguarBridge &bridge_;
    uint32_t id_;
    boost::shared_ptr<CANMessage> message_;
    boost::condition_variable cond_;
    boost::mutex mutex_;

    JaguarToken(JaguarBridge &bridge, uint32_t id);
    void unblock(boost::shared_ptr<CANMessage> message);

    friend class JaguarBridge;
};

};

#endif

/* vim set: sts=4 et sw=4 ts=4 : */
