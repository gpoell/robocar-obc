@dataclass(frozen=True)
class Topic:
    topic: str
    qos: int

    def get_values(self) -> tuple:
        vals = [val for attr, val in self.__dict__.items()]
        return tuple(vals)

@dataclass
class Publishers:

    def get_topics(self) -> list[tuple]:
        topics = [val for _, val in self.__dict__.items()]
        return [topic.get_values() for topic in topics]

@dataclass
class Subscribers:
    
    def get_topics(self) -> list[tuple]:
        topics = [val for _, val in self.__dict__.items()]
        return [topic.get_values() for topic in topics]


@dataclass
class Topics:
    publishers: Union[Publishers, None]
    subscribers: Union[Subscribers, None]

    def get_publishers(self) -> list[tuple]:
        if self.publishers == None:
            return [()]
        return self.publishers.get_topics()

    def get_subscribers(self) -> list[tuple]:
        if self.subscribers == None:
            return [()]
        return self.subscribers.get_topics()
